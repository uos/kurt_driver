// Michael Stypa - mstypa@uos.de

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "kurt2socketcan.h"

#define VERSION        2

#define ERRSOURCE      "kurt2socketcan"

#define CAN_CONTROL    0x00000001 // control message

#define CAN_INFO_1     0x00000004 // info message
#define CAN_ADC00_03   0x00000005 // analog input channels: 0 - 3
#define CAN_ADC04_07   0x00000006 // analog input channels: 4 - 7
#define CAN_ADC08_11   0x00000007 // analog input channels: 8 - 11
#define CAN_ADC12_15   0x00000008 // analog input channels: 12 - 15

// optional 2nd MC:
#define CAN_INFO_2     0x00000014 // info message
#define CAN_BDC00_03   0x00000015 // analog input channels: 0 - 3
#define CAN_BDC04_07   0x00000016 // analog input channels: 4 - 7
#define CAN_BDC08_11   0x00000017 // analog input channels: 8 - 11
#define CAN_BDC12_15   0x00000018 // analog input channels: 12 - 15

#define CAN_ENCODER    0x00000009 // 2 motor encoders
#define CAN_BUMPERC    0x0000000A // bumpers and remote control
#define CAN_DEADRECK   0x0000000B // position (dead reckoning)
#define CAN_GETSPEED   0x0000000C // current transl. and rot. speed
#define CAN_GETROTUNIT 0x00000010 // current rotunit angle

#define RAW            0          // raw control mode
#define SPEED          1          // speed control mode
#define SPEED_CM       2          // speed (cm/s) control mode
#define MC_RESET       0xFFFF	  // submit software reset

#define CAN_TILT_COMP  0x0000000D // data from tilt sensor
#define CAN_GYRO_MC1   0x0000000E // data from gyro connected to 1st C167
#define CAN_GYRO_MC2   0x0000001E // data from gyro connected to 2nd C167
#define C_FACTOR       10000      // correction factor needed for gyro_sigma

__u8 info_1[8];
__u8 info_2[8];
__u8 adc00_03[8], adc04_07[8], adc08_11[8], adc12_15[8];
__u8 bdc00_03[8], bdc04_07[8], bdc08_11[8], bdc12_15[8];
__u8 encoder[8];
__u8 bumperc[8];
__u8 rec_tilt_comp[8];
__u8 rec_gyro_mc1[8];
__u8 rec_gyro_mc2[8];
__u8 rec_deadreck[8];
__u8 rec_getspeed[8];
__u8 rec_getrotunit[8];

// encoder ticks
int left_encoder = 0, right_encoder = 0;
// msg number
int  nr_encoder_msg = 0;

char err[256];

// SocketCan specific variables
int cansocket; // can raw socket

char *send_frame(can_frame *frame) {
  int nbytes;
  if ((nbytes = write(cansocket, frame, sizeof(*frame))) != sizeof(*frame)) {
    sprintf(err, "%s: Error writing socket (%s)", ERRSOURCE, strerror(errno));
    return(err);
  }
  return(0);
}

char *receive_frame(can_frame *frame) {
  int nbytes, rc;
  fd_set rfds;
  struct timeval timeout;

  timeout.tv_sec = 5;
  timeout.tv_usec = 0;

  FD_ZERO(&rfds);
  FD_SET(cansocket, &rfds);

  rc = 1;

  rc = select(cansocket+1, &rfds, NULL, NULL, &timeout);

  if (rc == 0) {
    sprintf(err, "%s: Receiving frame timed out", ERRSOURCE);
    return(err);
  }
  else if (rc == -1) {
    sprintf(err, "%s: Error receiving frame (%d=%s)", ERRSOURCE, errno,
        strerror(errno));
    return(err);
  }

  nbytes = read(cansocket, frame, sizeof(*frame));
  //if(nbytes <= 0) fifo should be empty. but sadly this case never occurs.
  if (nbytes != sizeof(*frame)) {
    sprintf(err, "%s: Error reading socket (%s)", ERRSOURCE, strerror(errno));
    return(err);
  }

  return(0);
}

char *can_read_fifo(void) {
  struct can_frame frame;
  int i;
  char *err;

  while (42) {

    err = receive_frame(&frame);
    if (err) return(err);

    //TODO is this the right size of chunks or do
    //we have any chance to detect empty fifo?????
    if(frame.can_id == 10) break;

    switch (frame.can_id) {
      case CAN_CONTROL:
        break;
      case CAN_INFO_1:
        for (i = 0; i < frame.can_dlc; i++) {
          info_1[i] = frame.data[i];
        }
        break;
      case CAN_ADC00_03:
        for (i = 0; i < frame.can_dlc; i++) {
          adc00_03[i] = frame.data[i];
        }
        break;
      case CAN_ADC04_07:
        for (i = 0; i < frame.can_dlc; i++) {
          adc04_07[i] = frame.data[i];
        }
        break;
      case CAN_ADC08_11:
        for (i = 0; i < frame.can_dlc; i++) {
          adc08_11[i] = frame.data[i];
        }
        break;
      case CAN_ADC12_15:
        for (i = 0; i < frame.can_dlc; i++) {
          adc12_15[i] = frame.data[i];
        }
        break;
      case CAN_ENCODER:
        for (i = 0; i < frame.can_dlc; i++) {
          encoder[i] = frame.data[i];
        }
        nr_encoder_msg++;
        if (encoder[0] & 0x80) // negative Zahl auf 15 Bit genau
          left_encoder += (encoder[0] << 8) + encoder[1]-65536;
        else
          left_encoder += (encoder[0] << 8) + encoder[1];
        if (encoder[2] & 0x80) // negative Zahl auf 15 Bit genau
          right_encoder += (encoder[2] << 8) + encoder[3]-65536;
        else
          right_encoder += (encoder[2] << 8) + encoder[3];
/*
        printf("--- %ld %ld %d %d %d %d\n", left_encoder, right_encoder,
            encoder[0],
            encoder[1],
            encoder[2],
            encoder[3]);
        fflush(stdout);
*/
        break;
      case CAN_BUMPERC:
        for (i = 0; i < frame.can_dlc; i++) {
          bumperc[i] = frame.data[i];
        }
        break;
      case CAN_BDC00_03:
        for (i = 0; i < frame.can_dlc; i++) {
          bdc00_03[i] = frame.data[i];
        }
        break;
      case CAN_BDC04_07:
        for (i = 0; i < frame.can_dlc; i++) {
          bdc04_07[i] = frame.data[i];
        }
        break;
      case CAN_BDC08_11:
        for (i = 0; i < frame.can_dlc; i++) {
          bdc08_11[i] = frame.data[i];
        }
        break;
      case CAN_BDC12_15:
        for (i = 0; i < frame.can_dlc; i++) {
          bdc12_15[i] = frame.data[i];
        }
        break;
      case CAN_TILT_COMP:
        // printf("-");
        for (i = 0; i < frame.can_dlc; i++) {
          rec_tilt_comp[i] = frame.data[i];
        }
        break;
      case CAN_GYRO_MC1:
        //	 printf("CAN_GYRO_MC1");
        for (i = 0; i < frame.can_dlc; i++) {
          rec_gyro_mc1[i] = frame.data[i];
          //	 printf("%d \n",rec_gyro_mc1[i]);  
        }
        break;
      case CAN_GYRO_MC2:
        for (i = 0; i < frame.can_dlc; i++) {
          rec_gyro_mc2[i] = frame.data[i];
        }
        break;
      case CAN_DEADRECK:
        for (i = 0; i < frame.can_dlc; i++) {
          rec_deadreck[i] = frame.data[i];
        }
        break;
      case CAN_GETSPEED:
        for (i = 0; i < frame.can_dlc; i++) {
          rec_getspeed[i] = frame.data[i];
        }
        break;
      case CAN_GETROTUNIT:
        for (i = 0; i < frame.can_dlc; i++) {
          rec_getrotunit[i] = frame.data[i];
        }
        break;
      default:
        sprintf(err, "%s: Unknown CAN ID in can_read_fifo: %X", ERRSOURCE,
            frame.can_id);
        return(err);
    }
  }
  return(0);
}

char *can_init(int *version) {

  *version = VERSION;

  // initializing socketcan
  struct sockaddr_can addr;
  struct ifreq ifr;
  char caninterface[] = "can0"; //TODO automatic searching for caninterface
  int i;

  // prepare socket
  cansocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (cansocket < 0) {
    sprintf(err, "%s: Error opening socket (%s)", ERRSOURCE, strerror(errno));
    return(err);
  }

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, caninterface);
  if (ioctl(cansocket, SIOCGIFINDEX, &ifr) < 0) {
    sprintf(err, "%s: SIOCGIFINDEX for interace %s (%s)", ERRSOURCE,
        caninterface, strerror(errno));
    return(err);
  }
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(cansocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    sprintf(err, "%s: Error binding socket (%s)", ERRSOURCE, strerror(errno));
    return(err);
  }

  // initializing data structure
  for (i = 0; i < 8; i++) {
    info_1[i]   = 0;
    adc00_03[i] = 0;
    adc04_07[i] = 0;
    adc08_11[i] = 0;
    adc12_15[i] = 0;
    info_2[i]   = 0;
    bdc00_03[i] = 0;
    bdc04_07[i] = 0;
    bdc08_11[i] = 0;
    bdc12_15[i] = 0;
    encoder[i]  = 0;
    bumperc[i]  = 0;
    rec_tilt_comp[i] = 0;
    rec_gyro_mc1[i]  = 0;
    rec_gyro_mc2[i]  = 0;
    rec_deadreck[i]  = 0;
    rec_getspeed[i]  = 0;
    rec_getrotunit[i]  = 0;
  }

  left_encoder = right_encoder = 0;

  nr_encoder_msg = 0;

  return(0);
}

char *can_close(void) {
  close(cansocket);
  return(0);
}

char *can_motor(int left_pwm,  char left_dir,  char left_brake,
    int right_pwm, char right_dir, char right_brake) {

  struct can_frame frame;
  char *err;

  char left_dir_brake =  (left_dir << 1) + left_brake;
  char right_dir_brake = (right_dir << 1) + right_brake;

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = RAW >> 8;
  frame.data[1] = RAW;
  frame.data[2] = (left_dir_brake);
  frame.data[3] = (left_pwm >> 8);
  frame.data[4] = (left_pwm);
  frame.data[5] = (right_dir_brake);
  frame.data[6] = (right_pwm >> 8);
  frame.data[7] = (right_pwm);

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_speed(int left_speed, int right_speed) {
  struct can_frame frame;
  char *err;

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = SPEED >> 8;
  frame.data[1] = SPEED;
  frame.data[2] = left_speed >> 8;
  frame.data[3] = left_speed;
  frame.data[4] = right_speed >> 8;
  frame.data[5] = right_speed;
  frame.data[6] = 0;
  frame.data[7] = 0;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_speed_cm(int left_speed, int right_speed, int omega,
    int AntiWindup) {
  struct can_frame frame;
  char *err;
  //printf("vl: %d, vr: %d, omega: %d, aw: %d\n",left_speed, right_speed, omega, AntiWindup);
  int sign   = omega < 0      ? 1 : 0;
  int windup = AntiWindup > 0 ? 1 : 0;
  omega = abs(omega);
  omega = (omega << 1) + sign;
  omega = (omega << 1) + windup;

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = SPEED_CM >> 8;
  frame.data[1] = SPEED_CM;
  frame.data[2] = left_speed >> 8;
  frame.data[3] = left_speed;
  frame.data[4] = right_speed >> 8;
  frame.data[5] = right_speed;
  frame.data[6] = omega >> 8;
  frame.data[7] = omega;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_float(int mode, float f1, float f2) {
  struct can_frame frame;
  char *err;

  long l1 = (long)(1e6*f1);

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = (mode >> 8);
  frame.data[1] = mode;
  frame.data[2] = l1 >> 24;
  frame.data[3] = l1 >> 16;
  frame.data[4] = l1 >>  8;
  frame.data[5] = l1;
  frame.data[6] = 0;
  frame.data[7] = 0;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_mc_reset() {
  struct can_frame frame;
  char *err;

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = (MC_RESET >> 8);
  frame.data[1] = MC_RESET;
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_gyro_calibrate() {
  struct can_frame frame;
  char *err;

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = (__u8)(0xff);
  frame.data[1] = (0x02);
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_gyro_reset() {
  struct can_frame frame;
  char *err;

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = (0xff);
  frame.data[1] = (0x03);
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_ssc_reset() {
  struct can_frame frame;
  char *err;

  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = (0xff);
  frame.data[1] = (0x02);
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_rotunit_send(int speed) {
  struct can_frame frame;
  char *err;

  frame.can_id = 0x80;
  frame.can_dlc = 8;
  frame.data[0] = (speed >> 8); //high
  frame.data[1] = (speed & 0xFF); //low
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  err = send_frame(&frame);
  if (err) return(err);

  return(0);
}

char *can_sonar0_3(int *sonar0, int *sonar1, int *sonar2, int *sonar3,
    unsigned long *time) {

  char *err;

  err = can_read_fifo();
  if (err) return(err);

  *sonar0 = (adc00_03[0] << 8) + adc00_03[1];
  *sonar1 = (adc00_03[2] << 8) + adc00_03[3];
  *sonar2 = (adc00_03[4] << 8) + adc00_03[5];
  *sonar3 = (adc00_03[6] << 8) + adc00_03[7];
  *time = 0;

  return(0);
}

char *can_sonar4_7(int *sonar4, int *sonar5, int *sonar6, int *sonar7,
    unsigned long *time) {

  char *err;

  err = can_read_fifo();
  if (err) return(err);

  *sonar4 = (adc04_07[0] << 8) + adc04_07[1];
  *sonar5 = (adc04_07[2] << 8) + adc04_07[3];
  *sonar6 = (adc04_07[4] << 8) + adc04_07[5];
  *sonar7 = (adc04_07[6] << 8) + adc04_07[7];
  *time = 0;

  return(0);
}

char *can_sonar8_9(int *sonar8, int *sonar9, unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *sonar8 = (adc08_11[0] << 8) + adc08_11[1];
  *sonar9 = (adc08_11[2] << 8) + adc08_11[3];
  *time = 0;

  return(0);
}

char *can_tilt(int *tilt0, int *tilt1, unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *tilt0 = (adc08_11[4] << 8) + adc08_11[5];
  *tilt1 = (adc08_11[6] << 8) + adc08_11[7];
  *time = 0;
  return(0);
}

char *can_current(int *left, int *right, unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *left  = (adc12_15[0] << 8) + adc12_15[1];
  *right = (adc12_15[2] << 8) + adc12_15[3];
  *time = 0;
  return(0);
}

char *can_tilt_comp(double *tilt0, double *tilt1, int *comp1, int *comp2,
    unsigned long *time) {
  char *err;
  double a0, a1;
  unsigned int t0, t1;

  err = can_read_fifo();
  if (err) return(err);

  t0 = (rec_tilt_comp[0] << 8) + (rec_tilt_comp[1]);
  t1 = (rec_tilt_comp[2] << 8) + (rec_tilt_comp[3]);
  *comp1 = (rec_tilt_comp[4] << 8) + rec_tilt_comp[5];
  *comp2 = (rec_tilt_comp[6] << 8) + rec_tilt_comp[7];

  // calculate g values (offset and sensitivity correction)
  a0 = ((double)t0 - 32768.0) / 3932.0;
  a1 = ((double)t1 - 32768.0) / 3932.0;

  *tilt0 = asin(a0);	// calculate angles and convert do deg
  *tilt1 = asin(a1);

  *time = 0;
  return(0);
}

char *can_gyro_mc1(double *gyro_mc1_angle, double *gyro_mc1_sigma, unsigned long *time) {
  char *err;
  double angle, sigma_deg, tmp;
  signed long gyro_raw;

  err = can_read_fifo();
  if (err) return(err);

  gyro_raw = (rec_gyro_mc1[0] << 24) + (rec_gyro_mc1[1] << 16)
    + (rec_gyro_mc1[2] << 8)  + (rec_gyro_mc1[3]);
  angle = (double)gyro_raw / 4992511.0 * M_PI/180.0;
  if (angle >  M_PI) angle -= 2.0*M_PI;
  if (angle < -M_PI) angle += 2.0*M_PI;
  *gyro_mc1_angle = angle;

  sigma_deg = (double)((rec_gyro_mc1[4] << 24) + (rec_gyro_mc1[5] << 16)
      + (rec_gyro_mc1[6] << 8)  + (rec_gyro_mc1[7])) / C_FACTOR;
  tmp = (sqrt(sigma_deg)*M_PI/180.0);
  *gyro_mc1_sigma = tmp * tmp;

  return(0);
}

char *can_gyro_mc2(double *gyro_mc2_angle, double *gyro_mc2_sigma, unsigned long *time) {
  char *err;
  double angle, sigma_deg, tmp;
  signed long gyro_raw;

  err = can_read_fifo();
  if (err) return(err);

  gyro_raw = (rec_gyro_mc2[0] << 24) + (rec_gyro_mc2[1] << 16)
    + (rec_gyro_mc2[2] << 8)  + (rec_gyro_mc2[3]);
  angle = (double)gyro_raw / 4992511.0 * M_PI/180.0;
  if (angle >  M_PI) angle -= 2.0*M_PI;
  if (angle < -M_PI) angle += 2.0*M_PI;
  *gyro_mc2_angle = angle;

  sigma_deg = (double)((rec_gyro_mc2[4] << 24) + (rec_gyro_mc2[5] << 16)
      + (rec_gyro_mc2[6] << 8)  + (rec_gyro_mc2[7])) / C_FACTOR;
  tmp = (sqrt(sigma_deg)*M_PI/180.0);
  *gyro_mc2_sigma = tmp * tmp;

  return(0);
}

char *can_encoder(long *left, long *right, int *nr_msg) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *left = left_encoder;
  *right = right_encoder;
  *nr_msg = nr_encoder_msg;
  left_encoder = right_encoder = 0;
  nr_encoder_msg = 0;

  return(0);
}


char *can_rc(char *value, unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *value = bumperc[1];
  *time = 0;
  return(0);
}

char *can_bumper(char *value, unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *value = bumperc[0];
  *time = 0;
  return(0);
}

char *can_adc2nd0_3(int *adc2nd0, int *adc2nd1, int *adc2nd2, int *adc2nd3,
    unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *adc2nd0 = (bdc00_03[0] << 8) + bdc00_03[1];
  *adc2nd1 = (bdc00_03[2] << 8) + bdc00_03[3];
  *adc2nd2 = (bdc00_03[4] << 8) + bdc00_03[5];
  *adc2nd3 = (bdc00_03[6] << 8) + bdc00_03[7];
  *time = 0;
  return(0);
}

char *can_adc2nd4_7(int *adc2nd4, int *adc2nd5, int *adc2nd6, int *adc2nd7,
    unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *adc2nd4 = (bdc04_07[0] << 8) + bdc04_07[1];
  *adc2nd5 = (bdc04_07[2] << 8) + bdc04_07[3];
  *adc2nd6 = (bdc04_07[4] << 8) + bdc04_07[5];
  *adc2nd7 = (bdc04_07[6] << 8) + bdc04_07[7];
  *time = 0;
  return(0);
}

char *can_adc2nd8_11(int *adc2nd8, int *adc2nd9, int *adc2nd10, int *adc2nd11,
    unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *adc2nd8  = (bdc08_11[0] << 8) + bdc08_11[1];
  *adc2nd9  = (bdc08_11[2] << 8) + bdc08_11[3];
  *adc2nd10 = (bdc08_11[4] << 8) + bdc08_11[5];
  *adc2nd11 = (bdc08_11[6] << 8) + bdc08_11[7];
  *time = 0;
  return(0);
}

char *can_adc2nd12_15(int *adc2nd12, int *adc2nd13, int *adc2nd14, int *adc2nd15,
    unsigned long *time) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *adc2nd12 = (bdc12_15[0] << 8) + bdc12_15[1];
  *adc2nd13 = (bdc12_15[2] << 8) + bdc12_15[3];
  *adc2nd14 = (bdc12_15[4] << 8) + bdc12_15[5];
  *adc2nd15 = (bdc12_15[6] << 8) + bdc12_15[7];
  *time = 0;
  return(0);
}

char *can_sensor_state1(SENSOR_STATE *sensor_state1) {
  char *err;

  err = can_read_fifo();
  if (err) return(err);

  sensor_state1->sonar0 = (adc00_03[0] << 8) + adc00_03[1];
  sensor_state1->sonar1 = (adc00_03[2] << 8) + adc00_03[3];
  sensor_state1->sonar2 = (adc00_03[4] << 8) + adc00_03[5];
  sensor_state1->sonar3 = (adc00_03[6] << 8) + adc00_03[7];
  sensor_state1->time_adc00_03 = 0;

  sensor_state1->sonar4 = (adc04_07[0] << 8) + adc04_07[1];
  sensor_state1->sonar5 = (adc04_07[2] << 8) + adc04_07[3];
  sensor_state1->sonar6 = (adc04_07[4] << 8) + adc04_07[5];
  sensor_state1->sonar7 = (adc04_07[6] << 8) + adc04_07[7];
  sensor_state1->time_adc04_07 = 0;

  sensor_state1->sonar8  = (adc08_11[0] << 8) + adc08_11[1];
  sensor_state1->sonar9  = (adc08_11[2] << 8) + adc08_11[3];
  sensor_state1->tilt1   = (adc08_11[4] << 8) + adc08_11[5];
  sensor_state1->tilt2   = (adc08_11[6] << 8) + adc08_11[7];
  sensor_state1->time_adc08_11 = 0;

  sensor_state1->cur_left  = (adc12_15[0] << 8) + adc12_15[1];
  sensor_state1->cur_right = (adc12_15[2] << 8) + adc12_15[3];
  sensor_state1->compass1  = (adc12_15[4] << 8) + adc12_15[5];
  sensor_state1->compass2  = (adc12_15[6] << 8) + adc12_15[7];
  sensor_state1->time_adc12_15 = 0;

  sensor_state1->enc_left  = (encoder[0] << 8) + encoder[1];
  sensor_state1->enc_right = (encoder[2] << 8) + encoder[3];
  sensor_state1->time_encoder = 0;

  sensor_state1->bumper = bumperc[0];
  sensor_state1->rc     = bumperc[1];
  sensor_state1->time_bumper = 0;
  return(0);
}

char *can_sensor_state2(SENSOR_STATE *sensor_state2) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  sensor_state2->bdc[0] = (bdc00_03[0] << 8) + bdc00_03[1];
  sensor_state2->bdc[1] = (bdc00_03[2] << 8) + bdc00_03[3];
  sensor_state2->bdc[2] = (bdc00_03[4] << 8) + bdc00_03[5];
  sensor_state2->bdc[3] = (bdc00_03[6] << 8) + bdc00_03[7];
  sensor_state2->time_bdc00_03 = 0;

  sensor_state2->bdc[4] = (bdc04_07[0] << 8) + bdc04_07[1];
  sensor_state2->bdc[5] = (bdc04_07[2] << 8) + bdc04_07[3];
  sensor_state2->bdc[6] = (bdc04_07[4] << 8) + bdc04_07[5];
  sensor_state2->bdc[7] = (bdc04_07[6] << 8) + bdc04_07[7];
  sensor_state2->time_bdc04_07 = 0;

  sensor_state2->bdc[8]  = (bdc08_11[0] << 8) + bdc08_11[1];
  sensor_state2->bdc[9]  = (bdc08_11[2] << 8) + bdc08_11[3];
  sensor_state2->bdc[10] = (bdc08_11[4] << 8) + bdc08_11[5];
  sensor_state2->bdc[11] = (bdc08_11[6] << 8) + bdc08_11[7];
  sensor_state2->time_bdc08_11 = 0;

  sensor_state2->bdc[12] = (bdc12_15[0] << 8) + bdc12_15[1];
  sensor_state2->bdc[13] = (bdc12_15[2] << 8) + bdc12_15[3];
  sensor_state2->bdc[14] = (bdc12_15[4] << 8) + bdc12_15[5];
  sensor_state2->bdc[15] = (bdc12_15[6] << 8) + bdc12_15[7];
  sensor_state2->time_bdc12_15 = 0;
  return(0);
}

char *can_getrotunit(int *rot) {
  char *err;
  err = can_read_fifo();
  if (err) return(err);

  *rot = (rec_getrotunit[1] << 8) + rec_getrotunit[2];
  return(0);
}

char *can_time(unsigned long *time) {
  *time = 0;
  return(0);
}