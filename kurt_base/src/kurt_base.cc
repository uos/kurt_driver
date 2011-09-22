#include "kurt2socketcan.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include <signal.h>
#include <stdlib.h>

#include <cmath>
#include <string>
using std::string;
#include <algorithm>
using std::min;
using std::max;

// speed from encoder in m/s
double v_encoder_left = 0.0, v_encoder_right = 0.0, v_encoder = 0.0, v_encoder_angular = 0.0;
double sigma_x_, sigma_theta_, cov_x_y_, cov_x_theta_, cov_y_theta_;

// poses from encoder in m
double theta_from_encoder = 0.0, x_from_encoder = 0.0, z_from_encoder = 0.0;

// angle from gyro
double theta_from_gyro = 0.0;

bool use_gyrodometry;
// pose from gyrodometry in m
double theta_from_gyrodometry = 0.0, x_from_gyrodometry = 0.0, z_from_gyrodometry = 0.0;

// tilt information
double pitch = 0.0, roll = 0.0;

MOTOR_CTRL kurt2_ctrl;

//PWM data
const int MAX_V_LIST = 200;
const int nr_v = 1000;
double vmax;
int *pwm_v_l, *pwm_v_r;
double kp_l, kp_r; // schnell aenderung folgen
double ki_l, ki_r; // integrierer relative langsam
int leerlauf_adapt = 0;
double feedforward_turn; // in v = m/s
double turning_adaptation;

bool use_microcontroller;
bool use_rotunit;

int ticks_per_turn_of_wheel;
double wheel_perimeter;
double axis_length;

double wheelpos_l = 0.0, wheelpos_r = 0.0;

int ir_back, ir_right_back, ir_right, ir_right_front, ir_left_front, ir_left, ir_left_back;
int usound;

// values from Sharp GP2D12 IR ranger data sheet
#define IR_MIN 0.10 // [m]
#define IR_MAX 0.80 // [m]
#define IR_FOV 0.074859848 // [rad]

// values from Baumer UNDK30I6103 ultrasonic data sheet
#define SONAR_MIN 0.10 // [m]
#define SONAR_MAX 1.00 // [m]
#define SONAR_FOV 0.17809294 // [rad]

bool read_speed_to_pwm_leerlauf_tabelle(string &filename, int *nr, double **v_pwm_l, double **v_pwm_r);
void make_pwm_v_tab(int nr, double *v_pwm_l, double *v_pwm_r, int nr_v, int **pwm_v_l, int **pwm_v_r, double *v_max);
int k_can_init(void);
bool k_read_wheel_encoder(long *channel_1, long *channel_2, int *nr_msg);
bool odometry();
void populateCovariance(nav_msgs::Odometry &msg);
void get_gyro();
void get_ir(int *ir_back, int *ir_right_back, int *ir_right, int *ir_right_front, int *ir_left_front, int *ir_left, int *ir_left_back);
void get_sonar(int *s);
void normalize_ir(int *ir);
void normalize_sonar(int *s);
void get_tilt();
void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
void rotunitCallback(const geometry_msgs::Twist::ConstPtr& msg);
void k_can_close(void);
void set_wheel_speed2(double _v_l_soll, double _v_r_soll, double _v_l_ist, double _v_r_ist, double _omega,
                      double _AntiWindup);
void set_wheel_speed2_mc(double _v_l_soll, double _v_r_soll, double _omega, double _AntiWindup);

void quit(int sig)
{
  ROS_INFO("close");
  k_can_close();
  if (!use_microcontroller)
  {
    free(pwm_v_l);
    free(pwm_v_r);
  }
  exit(0);
}

int main(int argc, char** argv)
{
  double ki, kp;
  int rotunit_speed;
  string speedPwmLeerlaufTable;
  ros::init(argc, argv, "kurt_base");
  ros::NodeHandle n;
  ros::NodeHandle nh_ns("~");
  //Odometry parameter (defaults for kurt2 indoor)
  nh_ns.param("wheel_perimeter", wheel_perimeter, 37.9);
  nh_ns.param("axis_length", axis_length, 28.0);
  // parameter still in cm, converting to meter
  wheel_perimeter *= 0.01;
  axis_length *= 0.01;
  nh_ns.param("turning_adaptation", turning_adaptation, 0.69);
  nh_ns.param("ticks_per_turn_of_wheel", ticks_per_turn_of_wheel, 21950);

  nh_ns.param("use_gyrodometry", use_gyrodometry, false);
  nh_ns.param("use_rotunit", use_rotunit, false);

  nh_ns.param("x_stddev", sigma_x_, 0.002);
  nh_ns.param("rotation_stddev", sigma_theta_, 0.017);
  nh_ns.param("cov_xy", cov_x_y_, 0.0);
  nh_ns.param("cov_xrotation", cov_x_theta_, 0.0);
  nh_ns.param("cov_yrotation", cov_y_theta_, 0.0);

  use_microcontroller = true;

  //PID parameter (disables micro controller)
  if (nh_ns.getParam("speedtable", speedPwmLeerlaufTable))
  {
    use_microcontroller = false;
    nh_ns.param("feedforward_turn", feedforward_turn, 0.35);
    nh_ns.param("ki", ki, 3.4);
    nh_ns.param("kp", kp, 0.4);

    ki_l = ki_r = ki;
    kp_l = kp_r = kp;

    int nr;
    double *v_pwm_l, *v_pwm_r;
    if (!read_speed_to_pwm_leerlauf_tabelle(speedPwmLeerlaufTable, &nr, &v_pwm_l, &v_pwm_r))
    {
      return 2;
    }
    make_pwm_v_tab(nr, v_pwm_l, v_pwm_r, nr_v, &pwm_v_l, &pwm_v_r, &vmax);
    free(v_pwm_l);
    free(v_pwm_r);
  }

  bool publish_tf;
  string prefix_param;
  string tf_prefix;
  nh_ns.param("publish_tf", publish_tf, true);
  if (publish_tf)
  {
    n.searchParam("tf_prefix", prefix_param);
    n.getParam(prefix_param, tf_prefix);
  }

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState> ("joint_states", 1);

  sensor_msgs::JointState joint_state;
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] = "left_front_wheel_joint";
  joint_state.name[1] = "left_middle_wheel_joint";
  joint_state.name[2] = "left_rear_wheel_joint";
  joint_state.name[3] = "right_front_wheel_joint";
  joint_state.name[4] = "right_middle_wheel_joint";
  joint_state.name[5] = "right_rear_wheel_joint";

  if (use_rotunit)
  {
    nh_ns.param("InitialSpeed", rotunit_speed, 42);
    ros::Subscriber rot_vel_sub = n.subscribe("rot_vel", 10, rotunitCallback);

    joint_state.name.push_back("laser_rot_joint");
    joint_state.position.push_back(0.0);
  }

  ros::Rate loop_rate(100);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, velCallback);

  if (k_can_init() > 0)
  {
    ROS_ERROR("Can not init CAN");
    return 1;
  }
  /*
   Commented out during conversion to socketcan. For some reason,
   this doesn't work yet. However, there are enough other places
   that throw an error message if the communication doesn't work.

   long wheel_a, wheel_b;
   int nr_msg;
   nr_msg = 0;
   k_read_wheel_encoder(&wheel_a, &wheel_b, &nr_msg);
   if(!nr_msg) ROS_ERROR("Could not talk to chassi. Power On?");
   */
  if (use_rotunit)
  {
    can_rotunit_send(rotunit_speed);
  }

  signal(SIGINT, quit);
  ROS_INFO("started");

  ros::Time current_time;
  double x, y, th;
  int rot = 0;
  geometry_msgs::Quaternion odom_quat;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry> ("odom", 10);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher range_pub = n.advertise<sensor_msgs::Range> ("range", 10);

  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom_combined";
  odom.child_frame_id = "base_footprint";

  geometry_msgs::TransformStamped odom_trans;

  if (publish_tf)
  {
    odom_trans.header.frame_id = tf::resolve(tf_prefix, "odom_combined");
    odom_trans.child_frame_id = tf::resolve(tf_prefix, "base_footprint");
  }

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu> ("imu", 10);
  sensor_msgs::Imu imu;

  // this is intentionally base_link (the location of the imu) and not base_footprint,
  // but because they are connected by a fixed link, it doesn't matter
  imu.header.frame_id = "base_link";

  imu.angular_velocity_covariance[0] = -1; // no data avilable, see Imu.msg
  imu.linear_acceleration_covariance[0] = -1;

  sensor_msgs::Range range;

  while (ros::ok())
  {
    odometry();
    current_time = ros::Time::now();
    get_gyro();
    get_ir(&ir_back, &ir_right_back, &ir_right, &ir_right_front, &ir_left_front, &ir_left, &ir_left_back);
    get_sonar(&usound);
    //get_tilt();

    if (!use_gyrodometry)
    {
      x = z_from_encoder;
      y = -x_from_encoder;
      th = -theta_from_encoder;
    }
    else
    {
      x = z_from_gyrodometry;
      y = -x_from_gyrodometry;
      th = -theta_from_gyrodometry;
    }

    odom_quat = tf::createQuaternionMsgFromYaw(th);

    if (publish_tf)
    {
      odom_trans.header.stamp = current_time;
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      odom_broadcaster.sendTransform(odom_trans);
    }

    odom.header.stamp = current_time;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = v_encoder;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = v_encoder_angular;
    populateCovariance(odom);

    odom_pub.publish(odom);

    imu.header.stamp = current_time;
    imu.orientation = tf::createQuaternionMsgFromYaw(theta_from_gyro);
    imu_pub.publish(imu);

    normalize_sonar(&usound);
    range.header.stamp = current_time;
    range.header.frame_id = "ultrasound_front";
    range.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range.field_of_view = SONAR_FOV;
    range.min_range = SONAR_MIN;
    range.max_range = SONAR_MAX;
    range.range = usound/1000.0;
    range_pub.publish(range);

    normalize_ir(&ir_right_front);
    range.header.stamp = current_time;
    range.header.frame_id = "ir_right_front";
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.field_of_view = IR_FOV;
    range.min_range = IR_MIN;
    range.max_range = IR_MAX;
    range.range = ir_right_front/1000.0;
    range_pub.publish(range);

    normalize_ir(&ir_right);
    range.header.stamp = current_time;
    range.header.frame_id = "ir_right";
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.field_of_view = IR_FOV;
    range.min_range = IR_MIN;
    range.max_range = IR_MAX;
    range.range = ir_right/1000.0;
    range_pub.publish(range);

    normalize_ir(&ir_right_back);
    range.header.stamp = current_time;
    range.header.frame_id = "ir_right_back";
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.field_of_view = IR_FOV;
    range.min_range = IR_MIN;
    range.max_range = IR_MAX;
    range.range = ir_right_back/1000.0;
    range_pub.publish(range);

    normalize_ir(&ir_back);
    range.header.stamp = current_time;
    range.header.frame_id = "ir_back";
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.field_of_view = IR_FOV;
    range.min_range = IR_MIN;
    range.max_range = IR_MAX;
    range.range = ir_back/1000.0;
    range_pub.publish(range);

    normalize_ir(&ir_left_back);
    range.header.stamp = current_time;
    range.header.frame_id = "ir_left_back";
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.field_of_view = IR_FOV;
    range.min_range = IR_MIN;
    range.max_range = IR_MAX;
    range.range = ir_left_back/1000.0;
    range_pub.publish(range);

    normalize_ir(&ir_left);
    range.header.stamp = current_time;
    range.header.frame_id = "ir_left";
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.field_of_view = IR_FOV;
    range.min_range = IR_MIN;
    range.max_range = IR_MAX;
    range.range = ir_left/1000.0;
    range_pub.publish(range);

    normalize_ir(&ir_left_front);
    range.header.stamp = current_time;
    range.header.frame_id = "ir_left_front";
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.field_of_view = IR_FOV;
    range.min_range = IR_MIN;
    range.max_range = IR_MAX;
    range.range = ir_left_front/1000.0;
    range_pub.publish(range);

    joint_state.header.stamp = ros::Time::now();

    joint_state.position[0] = joint_state.position[1] = joint_state.position[2] = wheelpos_l;
    joint_state.position[3] = joint_state.position[4] = joint_state.position[5] = wheelpos_r;

    if (use_rotunit)
    {
      can_getrotunit(&rot);
      joint_state.position[6] = rot * 2 * M_PI / 10240;
    }

    // note: we reuse joint_state here, i.e., we modify joint_state after publishing.
    // this is only safe as long as nothing in the same process subscribes to the
    // joint_states topic. same for imu above.
    joint_pub.publish(joint_state);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double AntiWindup = 1.0;
  double v_l_soll = msg->linear.x - axis_length * msg->angular.z /*/ wheelRadius*/;
  double v_r_soll = msg->linear.x + axis_length * msg->angular.z/*/wheelRadius*/;

  if (msg->linear.x == 0 && msg->angular.z == 0)
  {
    AntiWindup = 0.0;
  }

  if (use_microcontroller)
  {
    set_wheel_speed2_mc(v_l_soll, v_r_soll, 0, AntiWindup);
  }
  else
  {
    set_wheel_speed2(v_l_soll, v_r_soll, v_encoder_left, v_encoder_right, 0, AntiWindup);
  }
}

void rotunitCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  can_rotunit_send(msg->angular.z);
}

void get_tilt()
{
  double tilt_lr, tilt_fb;
  unsigned long curr_time;
  int comp1_dummy, comp2_dummy;
  can_tilt_comp(&tilt_lr, &tilt_fb, &comp1_dummy, &comp2_dummy, &curr_time);
  tilt_lr = tilt_lr * 180.0 / M_PI;
  tilt_fb = tilt_fb * 180.0 / M_PI;
  roll = tilt_lr;
  pitch = tilt_fb;
}

void get_gyro()
{
  static bool gyro_offset_read = false;
  static double offset; // initial offset
  double theta, sigma; // sigma is not used because its undocumented..
  if (!gyro_offset_read)
  {
    can_gyro_reset();
    can_gyro_calibrate();
    can_gyro_mc1(&theta, &sigma, NULL);
    gyro_offset_read = true;
    sleep(10); // wait until gyro is stable
    can_gyro_mc1(&offset, &sigma, NULL);
    return;
  }
  can_gyro_mc1(&theta, &sigma, NULL);
  theta_from_gyro = theta - offset;
}

void gyrodometry(double local_dx, double local_dz)
{
  static double theta_alt_from_encoder = 0.0, theta_alt_from_gyro = 0.0;
  static double fehler_gyro = 0.0;

  // ist |dt_gyro| > |grenze_gyro| stellt der gyro eine kurve fest
  double grenze_gyro = 0.015;
  // ist |dt_odo| > |grenze_odo| stellt odometrie eine kurve fest
  double grenze_odo = 0.0006;

  double dt_odo = theta_alt_from_encoder - theta_from_encoder;
  double dt_gyro = theta_alt_from_gyro - theta_from_gyro;

  // machen Raeder eine Kurve?
  if (fabs(dt_odo) > grenze_odo)
  {
    // Fehler rausrechnen
    dt_gyro -= fehler_gyro;
    // Gyro zur winkelbestimmung nutzen
    theta_from_gyrodometry += dt_gyro;
  } // macht gyro eine kurve?
  else if (fabs(dt_gyro - fehler_gyro) > grenze_gyro)
  {
    // Fehler rausrechnen
    dt_gyro -= fehler_gyro;
    // Gyro zur Winkelbestimmung nutzen
    theta_from_gyrodometry += dt_gyro;
  } // sonst relativ geradeaus, korrigiere fehler_gyro
  else
  {
    double fehler_gyro_neu = dt_gyro - dt_odo;
    // alter Fehler wird gewichtet mit eingerechnet
    fehler_gyro = 0.6 * fehler_gyro_neu + 0.4 * fehler_gyro;
    theta_from_gyrodometry += dt_odo;
  }
  // Phasenkorrektur
  if (theta_from_gyrodometry > M_PI)
    theta_from_gyrodometry -= 2 * M_PI;
  if (theta_from_gyrodometry < -M_PI)
    theta_from_gyrodometry += 2 * M_PI;

  // x, z from gyro setzen
  x_from_gyrodometry += local_dx * cos(theta_alt_from_gyro) + local_dz * sin(theta_alt_from_gyro);
  z_from_gyrodometry += -local_dx * sin(theta_alt_from_gyro) + local_dz * cos(theta_alt_from_gyro);

  theta_alt_from_gyro = theta_from_gyro;
  theta_alt_from_encoder = theta_from_encoder;
}

bool odometry()
{
  long wheel_a = 0, wheel_b = 0;
  int nr_msg = 0; // msg count since the last readout of encoder ticks

  if (!k_read_wheel_encoder(&wheel_a, &wheel_b, &nr_msg))
  {
    return false;
  }

  // time_diff in sec; we hope kurt is precise ?? !! and sends every 10 ms
  double time_diff = nr_msg * 0.01;

  // store wheel position for published joint states
  wheelpos_l += 2.0 * M_PI * wheel_a / ticks_per_turn_of_wheel;
  if (wheelpos_l > M_PI)
    wheelpos_l -= 2.0 * M_PI;
  if (wheelpos_l < -M_PI)
    wheelpos_l += 2.0 * M_PI;

  wheelpos_r += 2 * M_PI * wheel_b / ticks_per_turn_of_wheel;
  if (wheelpos_r > M_PI)
    wheelpos_r -= 2.0 * M_PI;
  if (wheelpos_r < -M_PI)
    wheelpos_r += 2.0 * M_PI;

  // covered distance of wheels in meter
  double wheel_L = wheel_perimeter * wheel_a / ticks_per_turn_of_wheel;
  double wheel_R = wheel_perimeter * wheel_b / ticks_per_turn_of_wheel;

  // calc different speeds in meter / sec
  v_encoder_left = wheel_L / time_diff;
  v_encoder_right = wheel_R / time_diff;
  v_encoder = (v_encoder_right + v_encoder_left) * 0.5;
  // mehr Bahn als Winkelgeschwindigkeit
  v_encoder_angular = (v_encoder_right - v_encoder_left) * 0.5;

  // calc position deltas
  double local_dx, local_dz, dtheta_y = 0.0;
  const double EPSILON = 0.0001;

  if (fabs(wheel_L - wheel_R) < EPSILON)
  {
    if (fabs(wheel_L) < EPSILON)
    {
      local_dx = 0.0;
      local_dz = 0.0;
    }
    else // beide fast gleich (wheel_distance_a == wheel_distance_b)
    {
      local_dx = 0.0;
      local_dz = (wheel_L + wheel_R) * 0.5;
    }
  }
  else // (wheel_distance_a != wheel_distance_b) and > 0
  {
    double hypothenuse = 0.5 * (wheel_L + wheel_R);

    dtheta_y = (wheel_L - wheel_R) / axis_length * turning_adaptation;

    local_dx = hypothenuse * sin(dtheta_y);
    local_dz = hypothenuse * cos(dtheta_y);
  }

  // Odometrie : Koordinatentransformation in Weltkoordinaten
  x_from_encoder += local_dx * cos(theta_from_encoder) + local_dz * sin(theta_from_encoder);
  z_from_encoder += -local_dx * sin(theta_from_encoder) + local_dz * cos(theta_from_encoder);

  theta_from_encoder += dtheta_y;
  if (theta_from_encoder > M_PI)
    theta_from_encoder -= 2.0 * M_PI;
  if (theta_from_encoder < -M_PI)
    theta_from_encoder += 2.0 * M_PI;

  if (use_gyrodometry)
    gyrodometry(local_dx, local_dz);

  return true;
}

void populateCovariance(nav_msgs::Odometry &msg)
{
  double odom_multiplier = 1.0;

  if (fabs(v_encoder) <= 1e-8 && fabs(v_encoder_angular) <= 1e-8)
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = 1e-12;
    msg.twist.covariance[35] = 1e-12;

    msg.twist.covariance[30] = 1e-12;
    msg.twist.covariance[5] = 1e-12;
  }
  else
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = odom_multiplier * pow(sigma_x_, 2);
    msg.twist.covariance[35] = odom_multiplier * pow(sigma_theta_, 2);

    msg.twist.covariance[30] = odom_multiplier * cov_x_theta_;
    msg.twist.covariance[5] = odom_multiplier * cov_x_theta_;
  }

  msg.twist.covariance[7] = DBL_MAX;
  msg.twist.covariance[14] = DBL_MAX;
  msg.twist.covariance[21] = DBL_MAX;
  msg.twist.covariance[28] = DBL_MAX;

  msg.pose.covariance = msg.twist.covariance;

  if (fabs(v_encoder) <= 1e-8 && fabs(v_encoder_angular) <= 1e-8)
  {
    msg.pose.covariance[7] = 1e-12;

    msg.pose.covariance[1] = 1e-12;
    msg.pose.covariance[6] = 1e-12;

    msg.pose.covariance[31] = 1e-12;
    msg.pose.covariance[11] = 1e-12;
  }
  else
  {
    msg.pose.covariance[7] = odom_multiplier * pow(sigma_x_, 2) * pow(sigma_theta_, 2);

    msg.pose.covariance[1] = odom_multiplier * cov_x_y_;
    msg.pose.covariance[6] = odom_multiplier * cov_x_y_;

    msg.pose.covariance[31] = odom_multiplier * cov_y_theta_;
    msg.pose.covariance[11] = odom_multiplier * cov_y_theta_;
  }
}

/*****************************************************************************
 *          Auslesen der Zaehlerstaende auf der Encoder-Karte                *
 *****************************************************************************/
// This function has to be called 100 times per second
bool k_read_wheel_encoder(long *channel_1, long *channel_2, int *nr_msg)
{
  char *retext; // return text of CAN interface functions
  if ((retext = can_encoder(channel_1, channel_2, nr_msg)))
  {
    ROS_WARN("Error while reading encodervals (%s)", retext);
  }

  return *nr_msg > 0;
}

void transmit_speed(void)
{
  char *retext; // return text of CAN interface functions
  /*
   printf("### trans: %d %d %d %d %d %d\n",kurt2_ctrl.pwm_left,kurt2_ctrl.pwm_right,
   kurt2_ctrl.brake_left, kurt2_ctrl.brake_right,
   kurt2_ctrl.dir_left, kurt2_ctrl.dir_right);
   fflush(stdout);
   */
  if ((retext = can_motor(kurt2_ctrl.pwm_left, kurt2_ctrl.dir_left, kurt2_ctrl.brake_left, kurt2_ctrl.pwm_right,
                          kurt2_ctrl.dir_right, kurt2_ctrl.brake_right)))
  {
    ROS_WARN("transmit_speed %s", retext);
  }
}

// stoppen sofort
void k_hard_stop(void)
{
  kurt2_ctrl.control_mode = 0;
  kurt2_ctrl.pwm_left = 1023;
  kurt2_ctrl.dir_left = 0;
  kurt2_ctrl.brake_left = 1;
  kurt2_ctrl.pwm_right = 1023;
  kurt2_ctrl.dir_right = 0;
  kurt2_ctrl.brake_right = 1;
  transmit_speed();
}

int k_can_init(void)
{
  int can_version;
  char *retext; // return text of CAN interface functions

  if ((retext = can_init(&can_version)))
  {
    ROS_INFO("k_can_init %s", retext);
    return 1;
  }
  ROS_INFO("CAN interface version %3d.%02d",
      can_version / 100, can_version % 100);
  return 0;
}

void k_can_close(void)
{
  char *retext; // return text of CAN interface functions
  // be sure the robot ist stopped before closing the connection
  if (use_rotunit)
  {
    can_rotunit_send(0);
  }
  do
  {
    kurt2_ctrl.control_mode = 0;
    kurt2_ctrl.pwm_left = 1023;
    kurt2_ctrl.dir_left = 0;
    kurt2_ctrl.brake_left = 1;
    kurt2_ctrl.pwm_right = 1023;
    kurt2_ctrl.dir_right = 0;
    kurt2_ctrl.brake_right = 1;
  } while ((retext = can_motor(kurt2_ctrl.pwm_left, kurt2_ctrl.dir_left, kurt2_ctrl.brake_left, kurt2_ctrl.pwm_right,
                               kurt2_ctrl.dir_right, kurt2_ctrl.brake_right)));

  if ((retext = can_close()))
  {
    ROS_WARN("k_can_close %s", retext);
  }
}

// reads init data from pmw to speed experiment
bool read_speed_to_pwm_leerlauf_tabelle(string &filename, int *nr, double **v_pwm_l, double **v_pwm_r)
{
  int i;
  FILE *fpr_fftable = NULL;
  double t, v, vl, vr;
  int pwm;

  ROS_INFO("Open FeedForwardTabelle: %s", filename.c_str());
  fpr_fftable = fopen(filename.c_str(), "r");

  if (fpr_fftable == NULL)
  {
    ROS_ERROR("ERROR opening speedtable.");
    return false;
  }

  *nr = 1025; //TODO use global nr_v
  *v_pwm_l = (double *)calloc(*nr, sizeof(double));
  *v_pwm_r = (double *)calloc(*nr, sizeof(double));
  for (i = 0; i < *nr; i++)
  {
    //TODO ignoring return value
    fscanf(fpr_fftable, "%lf %lf %lf %lf %d", &t, &v, &vl, &vr, &pwm);
    (*v_pwm_l)[pwm] = vl;
    (*v_pwm_r)[pwm] = vr;
  }
  fclose(fpr_fftable);
  return true;
}

// generate reverse speedtable
void make_pwm_v_tab(int nr, double *v_pwm_l, double *v_pwm_r, int nr_v, int **pwm_v_l, int **pwm_v_r, double *v_max)
{
  int i, index_l, index_r;
  int last_l, last_r;
  double v_max_l = 0.0, v_max_r = 0.0;
  *v_max = 0.0;

  *pwm_v_l = (int *)calloc(nr_v + 1, sizeof(int));
  *pwm_v_r = (int *)calloc(nr_v + 1, sizeof(int));

  // check max wheel speed left right wheel
  for (i = 0; i < nr; i++)
  {
    if (v_pwm_l[i] > v_max_l)
      v_max_l = v_pwm_l[i];
    if (v_pwm_r[i] > v_max_r)
      v_max_r = v_pwm_r[i];
    *v_max = min(v_max_l, v_max_r);
  }

  // diskretisieren
  for (i = 0; i < nr; i++)
  {
    index_l = min(nr_v, max(0, (int)((v_pwm_l[i]) * (nr_v) / *v_max)));
    (*pwm_v_l)[index_l] = i;
    index_r = min(nr_v, max(0, (int)((v_pwm_r[i]) * (nr_v) / *v_max)));
    (*pwm_v_r)[index_r] = i;
  }
  (*pwm_v_l)[0] = 0; // for a stop
  (*pwm_v_r)[0] = 0;

  // interpolate blank entries
  last_l = (*pwm_v_l)[0];
  last_r = (*pwm_v_r)[0];
  for (i = 1; i < nr_v; i++)
  {
    // interpolation simple methode, pwm must be increasing
    if ((*pwm_v_l)[i] < (*pwm_v_l)[i - 1])
      (*pwm_v_l)[i] = (*pwm_v_l)[i - 1];
    if ((*pwm_v_r)[i] < (*pwm_v_r)[i - 1])
      (*pwm_v_r)[i] = (*pwm_v_r)[i - 1];
    /* interpolation advancde methode
     if ((*pwm_v_l)[i] == 0) {
     for (j = i+1; j <= nr_v; j++)
     if ((*pwm_v_l)[j] != 0) { next_l = (*pwm_v_l)[j]; j = nr_v; }
     (*pwm_v_l)[i] = (next_l + last_l ) / 2;
     } else last_l = (*pwm_v_l)[i];
     if ((*pwm_v_r)[i] == 0) {
     for (j = i+1; j <= nr_v; j++)
     if ((*pwm_v_r)[j] != 0) {next_r = (*pwm_v_r)[j];j = nr_v;}
     (*pwm_v_r)[i] = (next_r + last_r ) / 2;
     } else last_r = (*pwm_v_r)[i];
     */
  }
}

void print_v_pwm_tab(int nr, double *v_pwm_l, double *v_pwm_r)
{
  int i;

  printf("########## speed pwm tabelle #############\n");
  for (i = 0; i < nr; i++)
  {
    printf("%d %f %f\n", i, v_pwm_l[i], v_pwm_r[i]);
  }
  printf("##########################################\n");
}

void print_pwm_v_tab(int nr)
{
  int i;

  printf("######### pwm speed tabelle #############\n");
  for (i = 0; i < nr; i++)
  {
    printf("%d %f %d %d\n", i, vmax * i / nr, pwm_v_l[i], pwm_v_r[i]);
  }
  printf("vmax: %f ################################\n", vmax);
}

// PWM Lookup
// basis zuordnung
// darueber liegt ein PID geschwindigkeits regler
// Leerlauf adaption fuer den Teppich boden und geradeaus fahrt
void set_wheel_speed1(double v_l, double v_r, int integration_l, int integration_r)
{
  int index_l, index_r; // point in speed_pwm tabelle

  // calc pwm values form speed array
  index_l = (int)(fabs(v_l) / vmax * nr_v);
  index_r = (int)(fabs(v_r) / vmax * nr_v);

  index_l = min(nr_v - 1, index_l);
  index_r = min(nr_v - 1, index_r);

  // 1023 = zero, 0 = maxspeed
  if (fabs(v_l) > 0.01)
    kurt2_ctrl.pwm_left = max(0, min(1023, 1024 - pwm_v_l[index_l] - leerlauf_adapt - integration_l));
  else
    kurt2_ctrl.pwm_left = 1023;
  if (fabs(v_r) > 0.01)
    kurt2_ctrl.pwm_right = max(0, min(1023, 1024 - pwm_v_r[index_r] - leerlauf_adapt - integration_r));
  else
    kurt2_ctrl.pwm_right = 1023;

  //  printf(" pwm %d %d\n",kurt2_ctrl.pwm_left,kurt2_ctrl.pwm_right);

  if (v_l >= 0.0)
    kurt2_ctrl.dir_left = 0; // forward
  else
    kurt2_ctrl.dir_left = 1; // backward
  if (v_r >= 0.0)
    kurt2_ctrl.dir_right = 0; // s.o right wheel
  else
    kurt2_ctrl.dir_right = 1;

  kurt2_ctrl.control_mode = 0;
  // relase breaks
  kurt2_ctrl.brake_left = 0;
  kurt2_ctrl.brake_right = 0;

  transmit_speed();
}

void set_wheel_speed2_mc(double _v_l_soll, double _v_r_soll, double _omega, double _AntiWindup)
{
  char *retext; // return text of CAN interface functions
  // faktor um nachkommastellen mitzu übertragen, gesendet werden
  // nämlich nur integer werte ([+-]32e3 !)
  const long factor = 100;
  if ((retext = can_speed_cm((int)(_v_l_soll * factor), (int)(_v_r_soll * factor), (int)(factor * _omega / M_PI),
                             (int)_AntiWindup)))
  {
    ROS_INFO("set_wheel_speed2_mc %s", retext);
  }
}

// pid geschwindigkeits regler fuers linke und rechte rad
// omega wird benoetig um die integration fuer den darunterstehenden regler
// zu berechnen
void set_wheel_speed2(double _v_l_soll, double _v_r_soll, double _v_l_ist, double _v_r_ist, double _omega,
                      double _AntiWindup)
{
  int holdl = 0, holdr = 0; // delete errors

  // stellgroessen v=speed, l= links, r= rechts
  static double zl = 0.0, zr = 0.0;
  static double last_zl = 0.0, last_zr = 0.0;
  // regelabweichung e = soll - ist;
  static double el = 0.0, er = 0.0;
  static double last_el = 0.0, last_er = 0.0;
  // integral
  static double int_el = 0.0, int_er = 0.0;
  // differenzieren
  static double del = 0.0, der = 0.0;
  static double last_del = 0.0, last_der = 0.0;
  // zeitinterval
  double dt = 0.1;
  // filter fuer gueltige Werte
  static double last_v_l_ist = 0.0, last_v_r_ist = 0.0;
  // static int reached = 0;
  static double v_l_list[MAX_V_LIST], v_r_list[MAX_V_LIST];
  static int vl_index = 0, vr_index = 0;
  int i;
  static double f_v_l_ist, f_v_r_ist;
  // kd_l and kd_r allways 0 (using only pi controller here)
  double kd_l = 0.0, kd_r = 0.0; // nur pi regler d-anteil ausblenden

  //TODO use ros timer
  dt = 0.01;

  int_el *= _AntiWindup;
  int_er *= _AntiWindup;

  last_v_l_ist *= _AntiWindup;
  last_v_r_ist *= _AntiWindup;

  double turn_feedforward_l = -_omega / M_PI * feedforward_turn;
  double turn_feedforward_r = _omega / M_PI * feedforward_turn;

  // filtern: grosser aenderungen deuten auf fehlerhafte messungen hin
  if (fabs(_v_l_ist - last_v_l_ist) < 0.19)
  {
    // filter glaettung werte speichern
    v_l_list[vl_index] = _v_l_ist;
    f_v_l_ist = (v_l_list[vl_index] + v_l_list[vl_index - 1] + v_l_list[vl_index - 2] + v_l_list[vl_index - 3]) / 4.0;
    vl_index++; // achtung auf ueberlauf
    if (vl_index >= MAX_V_LIST)
    {
      vl_index = 3; // zum schutz vor ueberlauf kopieren bei hold einen mehr kopieren
      for (i = 0; i < 3; i++)
        v_l_list[2 - i] = v_l_list[MAX_V_LIST - i - 1];
    }

    el = _v_l_soll - f_v_l_ist;
    del = (el - last_el) / dt;
    int_el += el * dt;

    zl = kp_l * el + kd_l * del + ki_l * int_el + _v_l_soll + turn_feedforward_l;

    last_el = el; // last e
    last_del = del; // last de
    holdl = 0;
  }
  else
  {
    holdl = 1;
  }

  // filtern: grosser aenderungen deuten auf fehlerhafte messungen hin
  if (fabs(_v_r_ist - last_v_r_ist) < 0.19)
  {
    // filter glaettung werte speichern
    v_r_list[vr_index] = _v_r_ist;
    f_v_r_ist = (v_r_list[vr_index] + v_r_list[vr_index - 1] + v_r_list[vr_index - 2]) / 3.0;
    vr_index++; // achtung auf ueberlauf
    if (vr_index >= MAX_V_LIST)
    {
      vr_index = 3; // zum schutz vor ueberlauf kopieren bei hold einen mehr kopieren
      for (i = 0; i < 3; i++)
      {
        v_r_list[2 - i] = v_r_list[MAX_V_LIST - i - 1];
        ;
      }
    }

    er = _v_r_soll - f_v_r_ist;
    der = (er - last_er) / dt;
    int_er += er * dt;

    zr = kp_r * er + kd_r * der + ki_r * int_er + _v_r_soll + turn_feedforward_r;

    last_er = er; // last e
    last_der = der; // last de
    holdr = 0;
  }
  else
  {
    holdr = 1;
  }

  // range check und antiwindup stellgroessenbeschraenkung
  // verhindern das der integrier weiter hochlaeuft
  // deshalb die vorher addierten werte wieder abziehen
  if (zl > vmax)
  {
    zl = vmax;
    int_el -= el * dt;
  }
  if (zr > vmax)
  {
    zr = vmax;
    int_er -= er * dt;
  }
  if (zl < -vmax)
  {
    zl = -vmax;
    int_el -= el * dt;
  }
  if (zr < -vmax)
  {
    zr = -vmax;
    int_er -= er * dt;
  }

  // reduzieren

  double step_max = vmax * 0.5;
  /* kraft begrenzung damit die Kette nicht springt bzw
   der Motor ein wenig entlastet wird. bei vorgabe von max
   geschwindigkeit braucht es so 5 * 10 ms bevor die Maximale
   Kraft anliegt */
  if ((zl - last_zl) > step_max)
  {
    zl = last_zl + step_max;
  }
  if ((zl - last_zl) < -step_max)
  {
    zl = last_zl - step_max;
  }
  if ((zr - last_zr) > step_max)
  {
    zr = last_zr + step_max;
  }
  if ((zr - last_zr) < -step_max)
  {
    zr = last_zr - step_max;
  }

  // store old val for deviation plotting
  last_v_l_ist = _v_l_ist;
  last_v_r_ist = _v_r_ist;
  last_zl = zl;
  last_zr = zr;

  set_wheel_speed1(zl, zr, 0, 0);
}

void get_ir(int *ir_back, int *ir_right_back, int *ir_right, int *ir_right_front, int *ir_left_front, int *ir_left, int *ir_left_back)
{
  int dummy;
  unsigned long time;
  can_sonar0_3(ir_back, ir_right_back, ir_right_front, &dummy, &time);
  can_sonar4_7(ir_right_front, &dummy, ir_left_front, ir_left, &time);
  can_sonar8_9(&dummy, ir_left_back, &time);

  return;
}

void get_sonar(int *s)
{
  int dummy;
  unsigned long time;
  can_sonar4_7(&dummy, s, &dummy, &dummy, &time);
  return;
}

void normalize_ir(int *ir)
{
  if (*ir < IR_MIN * 1000 || *ir > IR_MAX * 1000) // convert m --> mm
  {
    *ir = -1;
    return;
  }
  //cout << (pow(30000.0 / ((double)*ir - 10.0), 1.0/1.4)- 10.0) << " ";
  *ir = (int)(pow(30000.0 / ((double)*ir - 10.0), 1.0 / 1.4) - 10.0);
  //cout << *ir << endl;
}

void normalize_sonar(int *s)
{
  if (*s < SONAR_MIN * 1000 || *s > SONAR_MAX * 1000) // convert m --> mm
  {
    *s = -1;
    return;
  }
  *s = (int)((double)*s * 0.110652 + 11.9231);
}
