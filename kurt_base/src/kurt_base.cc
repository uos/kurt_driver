#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include "kurt.h"
#include "comm.h"

class ROSComm : public Comm
{
  public:
    ROSComm(
        const ros::NodeHandle &n,
        double sigma_x,
        double sigma_theta,
        double cov_x_y,
        double cov_x_theta,
        double cov_y_theta,
        int ticks_per_turn_of_wheel) :
      n_(n),
      sigma_x_(sigma_x),
      sigma_theta_(sigma_theta),
      cov_x_y_(cov_x_y),
      cov_x_theta_(cov_x_theta),
      cov_y_theta_(cov_y_theta),
      ticks_per_turn_of_wheel_(ticks_per_turn_of_wheel),
      publish_tf_(false),
      odom_pub_(n_.advertise<nav_msgs::Odometry> ("odom", 10)),
      range_pub_(n_.advertise<sensor_msgs::Range> ("range", 10)),
      imu_pub_(n_.advertise<sensor_msgs::Imu> ("imu", 10)),
      joint_pub_(n_.advertise<sensor_msgs::JointState> ("joint_states", 1)) { }
    virtual void send_odometry(double z, double x, double theta, double
        v_encoder, double v_encoder_angular, int wheel_a, int wheel_b, double
        v_encoder_left, double v_encoder_right); virtual void
      send_sonar_leftBack(int ir_left_back);
    virtual void send_sonar_front_usound_leftFront_left(int ir_right_front, int
        usound, int ir_left_front, int ir_left);
    virtual void send_sonar_back_rightBack_rightFront(int ir_back, int
        ir_right_back, int ir_right);
    virtual void send_pitch_roll(double pitch, double roll);
    virtual void send_gyro(double theta, double sigma);
    virtual void send_rotunit(double rot);

    void setTFPrefix(const std::string &tf_prefix);

  private:
    void populateCovariance(nav_msgs::Odometry &msg, double v_encoder, double
        v_encoder_angular);

    ros::NodeHandle n_;
    double sigma_x_, sigma_theta_, cov_x_y_, cov_x_theta_, cov_y_theta_;
    int ticks_per_turn_of_wheel_;
    bool publish_tf_;
    std::string tf_prefix_;

    tf::TransformBroadcaster odom_broadcaster_;
    ros::Publisher odom_pub_;
    ros::Publisher range_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher joint_pub_;
};

void ROSComm::setTFPrefix(const std::string &tf_prefix)
{
  tf_prefix_ = tf_prefix;
}

void ROSComm::populateCovariance(nav_msgs::Odometry &msg, double v_encoder, double v_encoder_angular)
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

void ROSComm::send_odometry(double z, double x, double theta, double v_encoder, double v_encoder_angular, int wheel_a, int wheel_b, double v_encoder_left, double v_encoder_right)
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = tf::resolve(tf_prefix_, "odom_combined");
  odom.child_frame_id = tf::resolve(tf_prefix_, "base_footprint");

  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = z;
  odom.pose.pose.position.y = -x;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(-theta);

  odom.twist.twist.linear.x = v_encoder;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = v_encoder_angular;
  populateCovariance(odom, v_encoder, v_encoder_angular);

  odom_pub_.publish(odom);

  if (publish_tf_)
  {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = tf::resolve(tf_prefix_, "odom_combined");
    odom_trans.child_frame_id = tf::resolve(tf_prefix_, "base_footprint");

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = z;
    odom_trans.transform.translation.y = -x;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(-theta);

    odom_broadcaster_.sendTransform(odom_trans);
  }

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] = "left_front_wheel_joint";
  joint_state.name[1] = "left_middle_wheel_joint";
  joint_state.name[2] = "left_rear_wheel_joint";
  joint_state.name[3] = "right_front_wheel_joint";
  joint_state.name[4] = "right_middle_wheel_joint";
  joint_state.name[5] = "right_rear_wheel_joint";

  static double wheelpos_l = 0;
  wheelpos_l += 2.0 * M_PI * wheel_a / ticks_per_turn_of_wheel_;
  if (wheelpos_l > M_PI)
    wheelpos_l -= 2.0 * M_PI;
  if (wheelpos_l < -M_PI)
    wheelpos_l += 2.0 * M_PI;

  static double wheelpos_r = 0;
  wheelpos_r += 2 * M_PI * wheel_b / ticks_per_turn_of_wheel_;
  if (wheelpos_r > M_PI)
    wheelpos_r -= 2.0 * M_PI;
  if (wheelpos_r < -M_PI)
    wheelpos_r += 2.0 * M_PI;

  joint_state.position[0] = joint_state.position[1] = joint_state.position[2] = wheelpos_l;
  joint_state.position[3] = joint_state.position[4] = joint_state.position[5] = wheelpos_r;

  // note: we reuse joint_state here, i.e., we modify joint_state after publishing.
  // this is only safe as long as nothing in the same process subscribes to the
  // joint_states topic. same for imu above.
  joint_pub_.publish(joint_state);
}

void ROSComm::send_sonar_leftBack(int ir_left_back)
{
  sensor_msgs::Range range;
  range.header.stamp = ros::Time::now();

  range.header.frame_id = tf::resolve(tf_prefix_, "ir_left_back");
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_left_back / 100.0;
  range_pub_.publish(range);
}

void ROSComm::send_sonar_front_usound_leftFront_left(int ir_right_front, int usound, int ir_left_front, int ir_left)
{
  sensor_msgs::Range range;
  range.header.stamp = ros::Time::now();

  range.header.frame_id = tf::resolve(tf_prefix_, "ir_right_front");
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_right_front / 100.0;
  range_pub_.publish(range);

  range.header.frame_id = tf::resolve(tf_prefix_, "ultrasound_front");
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = SONAR_FOV;
  range.min_range = SONAR_MIN;
  range.max_range = SONAR_MAX;
  range.range = usound / 100.0;
  range_pub_.publish(range);

  range.header.frame_id = tf::resolve(tf_prefix_, "ir_left_front");
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_left_front / 100.0;
  range_pub_.publish(range);

  range.header.frame_id = tf::resolve(tf_prefix_, "ir_left");
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_left / 100.0;
  range_pub_.publish(range);
}

void ROSComm::send_sonar_back_rightBack_rightFront(int ir_back, int ir_right_back, int ir_right)
{
  sensor_msgs::Range range;
  range.header.stamp = ros::Time::now();

  range.header.frame_id = tf::resolve(tf_prefix_, "ir_back");
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_back / 100.0;
  range_pub_.publish(range);

  range.header.frame_id = tf::resolve(tf_prefix_, "ir_right_back");
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_right_back / 100.0;
  range_pub_.publish(range);

  range.header.frame_id = tf::resolve(tf_prefix_, "ir_right");
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_right / 100.0;
  range_pub_.publish(range);
}

void ROSComm::send_pitch_roll(double pitch, double roll)
{
  //TODO
}

void ROSComm::send_gyro(double theta, double sigma)
{
  sensor_msgs::Imu imu;

  // this is intentionally base_link (the location of the imu) and not base_footprint,
  // but because they are connected by a fixed link, it doesn't matter
  imu.header.frame_id = tf::resolve(tf_prefix_, "base_link");
  imu.header.stamp = ros::Time::now();

  imu.angular_velocity_covariance[0] = -1; // no data avilable, see Imu.msg
  imu.linear_acceleration_covariance[0] = -1;

  imu.orientation = tf::createQuaternionMsgFromYaw(theta);
  imu.orientation_covariance[0] = sigma;
  imu.orientation_covariance[4] = sigma;
  imu.orientation_covariance[8] = sigma;
  imu_pub_.publish(imu);
}

void ROSComm::send_rotunit(double rot)
{
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(1);
  joint_state.position.resize(1);
  joint_state.name[0] = "laser_rot_joint";
  joint_state.position[0] = rot;

  joint_pub_.publish(joint_state);
}

class ROSCall
{
  public:
    ROSCall(Kurt &kurt, double axis_length) :
      kurt_(kurt),
      axis_length_(axis_length),
      v_l_soll_(0.0),
      v_r_soll_(0.0),
      AntiWindup_(1.0),
      last_cmd_vel_time_(0.0) { }
    void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void pidCallback(const ros::TimerEvent& event);
    void rotunitCallback(const geometry_msgs::Twist::ConstPtr& msg);

  private:
    Kurt &kurt_;
    double axis_length_;
    double v_l_soll_;
    double v_r_soll_;
    double AntiWindup_;
    ros::Time last_cmd_vel_time_;
};

void ROSCall::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  AntiWindup_ = 1.0;
  last_cmd_vel_time_ = ros::Time::now();
  v_l_soll_ = msg->linear.x - axis_length_ * msg->angular.z /*/ wheelRadius*/;
  v_r_soll_ = msg->linear.x + axis_length_ * msg->angular.z/*/wheelRadius*/;

  if (msg->linear.x == 0 && msg->angular.z == 0)
  {
    AntiWindup_ = 0.0;
  }
}

void ROSCall::pidCallback(const ros::TimerEvent& event)
{
  double v_l_soll = 0.0;
  double v_r_soll = 0.0;
  double AntiWindup = 1.0;

  if (ros::Time::now() - last_cmd_vel_time_ < ros::Duration(0.6))
  {
    v_l_soll = v_l_soll_;
    v_r_soll = v_r_soll_;
    AntiWindup = AntiWindup_;
  }

  kurt_.set_wheel_speed(v_l_soll, v_r_soll, AntiWindup);
}

void ROSCall::rotunitCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    kurt_.can_rotunit_send(msg->angular.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kurt_base");
  ros::NodeHandle n;
  ros::NodeHandle nh_ns("~");

  //Odometry parameter (defaults for kurt2 indoor)
  double wheel_perimeter;
  nh_ns.param("wheel_perimeter", wheel_perimeter, 0.379);
  double axis_length;
  nh_ns.param("axis_length", axis_length, 0.28);

  double turning_adaptation;
  nh_ns.param("turning_adaptation", turning_adaptation, 0.69);
  int ticks_per_turn_of_wheel;
  nh_ns.param("ticks_per_turn_of_wheel", ticks_per_turn_of_wheel, 21950);

  double sigma_x, sigma_theta, cov_x_y, cov_x_theta, cov_y_theta;
  nh_ns.param("x_stddev", sigma_x, 0.002);
  nh_ns.param("rotation_stddev", sigma_theta, 0.017);
  nh_ns.param("cov_xy", cov_x_y, 0.0);
  nh_ns.param("cov_xrotation", cov_x_theta, 0.0);
  nh_ns.param("cov_yrotation", cov_y_theta, 0.0);

  ROSComm roscomm(n, sigma_x, sigma_theta, cov_x_y, cov_x_theta, cov_y_theta, ticks_per_turn_of_wheel);

  Kurt kurt(roscomm, wheel_perimeter, axis_length, turning_adaptation, ticks_per_turn_of_wheel);

  //PID parameter (disables micro controller)
  std::string speedPwmLeerlaufTable;
  if (nh_ns.getParam("speedtable", speedPwmLeerlaufTable))
  {
    double feedforward_turn;
    nh_ns.param("feedforward_turn", feedforward_turn, 0.35);
    double ki, kp;
    nh_ns.param("ki", ki, 3.4);
    nh_ns.param("kp", kp, 0.4);
    if (!kurt.setPWMData(speedPwmLeerlaufTable, feedforward_turn, ki, kp))
      return 1;
  }

  bool use_rotunit;
  nh_ns.param("use_rotunit", use_rotunit, false);
  if (use_rotunit) {
    double rotunit_speed;
    nh_ns.param("rotunit_speed", rotunit_speed, M_PI/6.0);
    kurt.can_rotunit_send(rotunit_speed);
  }

  bool publish_tf;
  nh_ns.param("publish_tf", publish_tf, false);
  std::string tf_prefix;
  tf_prefix = tf::getPrefixParam(nh_ns);
  roscomm.setTFPrefix(tf_prefix);

  ROSCall roscall(kurt, axis_length);

  ros::Timer pid_timer;
  ros::Subscriber cmd_vel_sub;
  
  bool use_kurt;
  nh_ns.param("use_kurt", use_kurt, true);

  if (use_kurt){
    pid_timer = n.createTimer(ros::Duration(0.01), &ROSCall::pidCallback, &roscall);
    cmd_vel_sub = n.subscribe("cmd_vel", 10, &ROSCall::velCallback, &roscall);
  }
  ros::Subscriber rot_vel_sub;
  if (use_rotunit)
    rot_vel_sub = n.subscribe("rot_vel", 10, &ROSCall::rotunitCallback, &roscall);

  while (ros::ok())
  {
    kurt.can_read_fifo();
    ros::spinOnce();
  }

  return 0;
}
