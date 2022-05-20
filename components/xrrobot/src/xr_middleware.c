#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

//  #define debug  1

u_char Asyn = 1;  // 1:Start receiving and parsing data
float xr_imu_list[9] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0};  //数据格式[陀螺仪Roll，陀螺仪Pitch，陀螺仪Yaw，线加速度X，线加速度Y，线加速度Z，姿态角Roll，姿态角Pitch，姿态角Yaw]
float xr_odom_list[6] = {
    0,
    0,
    0,
    0,
    0,
    0};  //数据格式(list)[里程计X坐标，里程计Y坐标，里程计Yaw角度，机器人X轴线速度，机器人Y轴线速度，机器人Yaw轴角度]
u_int16_t xr_wheelspeed_list[9] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0};  //数据格式(list)[A轮转速，B轮转速，C轮转速，D轮转速，
         //A轮目标转速，B轮目标转速，C轮目标转速，D轮目标转速]
float xr_vol = 7.4;  //电池电压

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
  struct termios newttys1, oldttys1;

  /*保存原有串口配置*/
  if (tcgetattr(fd, &oldttys1) != 0) {
    perror("Setupserial 1\r\n");
    return -1;
  }
  bzero(&newttys1, sizeof(newttys1));
  newttys1.c_cflag |=
      (CLOCAL | CREAD); /*CREAD 开启串行数据接收，CLOCAL并打开本地连接模式*/

  newttys1.c_cflag &= ~CSIZE; /*设置数据位*/
  /*数据位选择*/
  switch (nBits) {
    case 7:
      newttys1.c_cflag |= CS7;
      break;
    case 8:
      newttys1.c_cflag |= CS8;
      break;
  }

  /*设置奇偶校验位*/
  switch (nEvent) {
    case '0':                     /*奇校验*/
      newttys1.c_cflag |= PARENB; /*开启奇偶校验*/
      newttys1.c_iflag |=
          (INPCK |
           ISTRIP); /*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
      newttys1.c_cflag |= PARODD; /*启用奇校验(默认为偶校验)*/
      break;
    case 'E':                     /*偶校验*/
      newttys1.c_cflag |= PARENB; /*开启奇偶校验  */
      newttys1.c_iflag |=
          (INPCK | ISTRIP); /*打开输入奇偶校验并去除字符第八个比特*/
      newttys1.c_cflag &= ~PARODD; /*启用偶校验*/
      break;
    case 'N': /*无奇偶校验*/
      newttys1.c_cflag &= ~PARENB;
      break;
  }
  newttys1.c_oflag &= ~(INLCR | IGNCR | ICRNL | ONLCR | OCRNL);
  newttys1.c_iflag &= ~IXON;
  newttys1.c_iflag &= ~IXOFF;
  /*设置波特率*/
  switch (nSpeed) {
    case 2400:
      cfsetispeed(&newttys1, B2400);
      cfsetospeed(&newttys1, B2400);
      break;
    case 4800:
      cfsetispeed(&newttys1, B4800);
      cfsetospeed(&newttys1, B4800);
      break;
    case 9600:
      cfsetispeed(&newttys1, B9600);
      cfsetospeed(&newttys1, B9600);
      break;
    case 115200:
      cfsetispeed(&newttys1, B115200);
      cfsetospeed(&newttys1, B115200);
      break;
    case 57600:
      cfsetispeed(&newttys1, B57600);
      cfsetospeed(&newttys1, B57600);
      break;
    case 230400:
      cfsetispeed(&newttys1, B230400);
      cfsetospeed(&newttys1, B230400);
      break;
    case 2500000:
      cfsetispeed(&newttys1, B2500000);
      cfsetospeed(&newttys1, B2500000);
      break;
    case 500000:
      cfsetispeed(&newttys1, B500000);
      cfsetospeed(&newttys1, B500000);
      break;
    case 921600:
      cfsetispeed(&newttys1, B921600);
      cfsetospeed(&newttys1, B921600);
      break;
    case 1000000:
      cfsetispeed(&newttys1, B1000000);
      cfsetospeed(&newttys1, B1000000);
      break;
    default:
      printf(
          "set band %d error,use default 115200 \rpls choose from "
          "2400,4800,9600,57600,115200,2500000,500000,921600,1000000\r\n",
          nSpeed);
      cfsetispeed(&newttys1, B115200);
      cfsetospeed(&newttys1, B115200);
      break;
  }
  /*设置停止位*/
  if (nStop ==
      1) /*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB*/
  {
    newttys1.c_cflag &= ~CSTOPB; /*默认为一位停止位； */
  } else if (nStop == 2) {
    newttys1.c_cflag |= CSTOPB; /*CSTOPB表示送两位停止位*/
  }

  // newttys1.c_cflag &= ~BRKINT;//如果设置了IGNBRK，BREAK键输入将被忽略
  // 0x13丢失

  /*设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时*/
  newttys1.c_cc[VTIME] = 0; /*非规范模式读取时的超时时间；*/
  newttys1.c_cc[VMIN] = 0;  /*非规范模式读取时的最小字符数*/
  tcflush(fd, TCIFLUSH); /*tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来
                          */

  /*激活配置使其生效*/
  if ((tcsetattr(fd, TCSANOW, &newttys1)) != 0) {
    perror("com set error\r\n");
    return -1;
  }

  return 0;
}

int xr_send_data(int fd, u_char *send_data, u_int16_t len) {
  int rv = -1;
  rv = write(fd, send_data, len);
  if (rv < 0) {
    printf("write error:%s\r\n", strerror(errno));
    close(fd);
    return -1;
  }
#ifdef debug
  printf("send_data numbers %d OK! \r\n", rv);
#endif
  return 0;
}

u_int16_t set_s16(int value)  // ENC 16bit Signed number
{
  u_int16_t temp = 0;
  if (value >= 0) {
    temp = value & 0xffff;
  } else {
    temp = (value + 0x10000) & 0xffff;
  }
  return temp;
}

int get_s16(int value)  // DEC 16bit Signed number
{
  int temp = 0;
  temp = value & 0xFFFF;
  if (temp < 0x8000) {
    return temp;
  } else {
    return temp - 0x10000;
  }
}

int get_s24(int value)  // DEC 24bit Signed number
{
  int temp = 0;
  temp = value & 0xFFFFFF;
  if (temp < 0x800000) {
    return temp;
  } else {
    return temp - 0x1000000;
  }
}

int check_crc(u_char *buf, u_int16_t length) {
  // TYPE, CTL, LEN, DATA_BUF, CRC
  // u_int8_t length = sizeof(buf)/sizeof(buf[0]);
  u_char *temp;
  u_char CRC = 0;
  u_char crc = 0;
  u_int16_t crc_sum = 0;

#ifdef debug
  printf("length is %d\r\n", length);
  for (size_t i = 0; i < length; i++) {
    printf("got data %ld is %d\r\n", i, buf[i]);
  }
#endif

  if (length < 4) {
#ifdef debug
    printf("numbers less  than 4\r\n");
    printf("crc check failed!\r\n");
#endif

    return 1;

  } else if (length != buf[2] + 4) {
#ifdef debug
    printf("data formate is [TYPE, CTL, LEN, DATA_BUF, CRC]\r\n");
    printf("length != LEN+4\r\n");
    printf("crc check failed!\r\n");
#endif

    return 2;
  } else {
    temp =
        (u_int8_t *)malloc(length * sizeof(u_int8_t));  //为数组temp动态分配内存

    memset(temp, 0, length);
    memcpy(temp, buf, length * sizeof(u_int8_t));

    for (size_t i = 0; i < length - 1; i++) {
      // printf("%d\r\n",buf[i]);
      // printf("%d\r\n",temp[i]);
      crc_sum += temp[i];
    }
    crc = (crc_sum / (length - 1)) & 0xFF;
    CRC = temp[temp[2] + 3];
    free(temp);

    if (crc == CRC) {
#ifdef debug
      printf("crc got is %d, decode is %d,check pass!\r\n", CRC, crc);
#endif
      return 0;
    } else {
#ifdef debug
      printf("crc got is %d, decode is %d,check failed!\r\n", CRC, crc);
#endif
      return 3;
    }
  }
}

int xr_encode(u_char type,
              u_char ctl,
              u_char *data_buf,
              u_int16_t data_length,
              u_char *send_data) {
  u_int16_t crc_sum = 0;
  u_int16_t len = data_length + 8;
  u_char crc = 0;
  u_char *temp;
  u_char head[] = {0XFF, 0XFF};
  u_char tail[] = {0xEE, 0xEE};
  // u_char send_data[len];
  temp = (u_int8_t *)malloc(len * sizeof(u_int8_t));  //为数组temp动态分配内存
  crc_sum = type + ctl + data_length;
  for (size_t i = 0; i < data_length; i++) {
    /* code */
    crc_sum += data_buf[i];
  }
  crc = (crc_sum / (data_length + 3)) & 0xff;

  memset(temp, 0, len);
  memcpy(temp, head, 2);
  memset(temp + 2, type, 1);         // type
  memset(temp + 3, ctl, 1);          // ctl
  memset(temp + 4, data_length, 1);  // len
  memcpy(temp + 5, data_buf, data_length);
  memset(temp + len - 3, crc, 1);  // crc
  memcpy(temp + len - 2, tail, 2);
  memcpy(send_data, temp, len);
#ifdef debug
  printf("xr_encode...\r\n");
  printf("send_data length is %d\r\n", len);
  for (size_t i = 0; i < len; i++) {
    printf("send_data %ld is 0x%02x\r\n", i, temp[i]);
  }
#endif
  free(temp);

  return 0;
}

int _XRMiddleWare_init(int fd)  //开始获取数据
{
  int rv = -1;
  u_char type = 0x03;
  u_char ctl = 0x01;
  u_char data_buf[] = {0};
  u_char data_length = 0;
  u_char len = data_length + 8;
  u_char *send_data;
  // tcflush(fd, TCIOFLUSH);//清空缓存
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
    Asyn = 1;
#ifdef debug
    printf("send init cmd OK! The driverboard starts uploading data!\r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send init cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_deinit(int fd)  //停止数据上传
{
  int rv = -1;
  u_char type = 0x03;
  u_char ctl = 0x02;
  u_char data_buf[] = {0};
  u_char data_length = 0;
  u_char len = data_length + 8;
  u_char *send_data;
  tcflush(fd, TCIOFLUSH);  //清空缓存
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
    Asyn = 0;
#ifdef debug
    printf("send deinit cmd OK! The driverboard stops uploading data!\r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send deinit cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_SetPID(int fd,
                         u_int16_t P,
                         u_int16_t I,
                         u_int16_t D)  //设置PID参数
{
  int rv = -1;
  u_char type = 0x04;
  u_char ctl = 0x01;
  u_char data_length = 6;
  u_char data_buf[data_length];
  u_char len = data_length + 8;
  u_char *send_data;

  data_buf[0] = set_s16(P) >> 8;
  data_buf[1] = set_s16(P) & 0xFF;
  data_buf[2] = set_s16(I) >> 8;
  data_buf[3] = set_s16(I) & 0xFF;
  data_buf[4] = set_s16(D) >> 8;
  data_buf[5] = set_s16(D) & 0xFF;
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
#ifdef debug
    printf("send PID cmd OK! \r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send PID cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_set_encoder_resolution(
    int fd, u_int16_t encoder_resolution)  //分辨率 1320
{
  int rv = -1;
  u_char type = 0x04;
  u_char ctl = 0x02;
  u_char data_length = 2;
  u_char data_buf[data_length];
  u_char len = data_length + 8;
  u_char *send_data;

  data_buf[0] = set_s16(encoder_resolution) >> 8;
  data_buf[1] = set_s16(encoder_resolution) & 0xFF;
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
#ifdef debug
    printf("send encoder_resolution cmd OK! \r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send encoder_resolution cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_set_wheel_diameter(
    int fd, float wheel_radius)  //轮子直径0.097m,半径/2
{
  int rv = -1;
  u_char type = 0x04;
  u_char ctl = 0x03;
  u_char data_length = 2;
  u_char data_buf[data_length];
  u_char len = data_length + 8;
  u_char *send_data;
  int value = 0;
  value = (int)(wheel_radius * 1000);

  data_buf[0] = set_s16(value) >> 8;
  data_buf[1] = set_s16(value) & 0xFF;
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
#ifdef debug
    printf("send wheel_radius cmd OK! \r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send wheel_radius cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_set_wheel_track(int fd, float wheel_track_div2)  //轮距 0.11m
{
  int rv = -1;
  u_char type = 0x04;
  u_char ctl = 0x04;
  u_char data_length = 2;
  u_char data_buf[data_length];
  u_char len = data_length + 8;
  u_char *send_data;
  int value = 0;
  value = (int)(wheel_track_div2 * 1000);

  data_buf[0] = set_s16(value) >> 8;
  data_buf[1] = set_s16(value) & 0xFF;
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
#ifdef debug
    printf("send wheel_track_div2 cmd OK! \r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send wheel_track_div2 cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_set_half_long(int fd, float half_long)  //前后 0.095m
{
  int rv = -1;
  u_char type = 0x04;
  u_char ctl = 0x05;
  u_char data_length = 2;
  u_char data_buf[data_length];
  u_char len = data_length + 8;
  u_char *send_data;
  int value = 0;
  value = (int)(half_long * 1000);

  data_buf[0] = set_s16(value) >> 8;
  data_buf[1] = set_s16(value) & 0xFF;
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
#ifdef debug
    printf("send half_long cmd OK! \r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send half_long cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_SetVelocity(
    int fd,
    float X,
    float Y,
    float yaw)  //设定速度参数，x，y，单位为m/s，yaw为 rad/s
{
  float x = -Y;
  float y = X;
  int rv = -1;
  u_char type = 0x02;
  u_char ctl = 0x00;
  u_char data_length = 6;
  u_char data_buf[data_length];
  u_char len = data_length + 8;
  u_char *send_data;
  int value = 0;

  value = (int)(x * 1000);
  data_buf[0] = set_s16(value) >> 8;
  data_buf[1] = set_s16(value) & 0xFF;

  value = (int)(y * 1000);
  // printf("%02x",value);
  data_buf[2] = set_s16(value) >> 8;
  data_buf[3] = set_s16(value) & 0xFF;

  value = (int)(yaw * 1000);
  // printf("%02x",value);
  data_buf[4] = set_s16(value) >> 8;
  data_buf[5] = set_s16(value) & 0xFF;

  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
#ifdef debug
    printf("send SetVelocity cmd OK! \r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send SetVelocity cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_SetWheelSpeed(
    int fd,
    u_int16_t w1,
    u_int16_t w2,
    u_int16_t w3,
    u_int16_t w4)  //设定转速参数，w1, w2, w3, w4，round/s
{
  int rv = -1;
  u_char type = 0x01;
  u_char ctl = 0x00;
  u_char data_length = 8;
  u_char data_buf[data_length];
  u_char len = data_length + 8;
  u_char *send_data;

  data_buf[0] = set_s16(w1) >> 8;
  data_buf[1] = set_s16(w1) & 0xFF;
  data_buf[2] = set_s16(w2) >> 8;
  data_buf[3] = set_s16(w2) & 0xFF;
  data_buf[4] = set_s16(w3) >> 8;
  data_buf[5] = set_s16(w3) & 0xFF;
  data_buf[6] = set_s16(w4) >> 8;
  data_buf[7] = set_s16(w4) & 0xFF;

  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
#ifdef debug
    printf("send SetWheelSpeed cmd OK! \r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send SetWheelSpeed cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_clrOdom(int fd)  //清空里程计
{
  int rv = -1;
  u_char type = 0x08;
  u_char ctl = 0x01;
  u_char data_buf[] = {0};
  u_char data_length = 0;
  u_char len = data_length + 8;
  u_char *send_data;
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
    Asyn = 1;
#ifdef debug
    printf("send clrOdom cmd OK! The driverboard starts uploading data!\r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send clrOdom cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

int _XRMiddleWare_LowPower(int fd)  //蜂鸣器警报
{
  int rv = -1;
  u_char type = 0x06;
  u_char ctl = 0x01;
  u_char data_buf[] = {0};
  u_char data_length = 0;
  u_char len = data_length + 8;
  u_char *send_data;
  send_data =
      (u_char *)malloc(len * sizeof(u_char));  //为数组send_data动态分配内存
  xr_encode(type, ctl, data_buf, data_length, send_data);
  rv = xr_send_data(fd, send_data, len);
  free(send_data);
  if (!rv) {
    Asyn = 1;
#ifdef debug
    printf("send LowPower cmd OK!\r\n");
#endif
    return 0;
  } else {
#ifdef debug
    printf("send LowPower cmd failed! pls check!\r\n");
#endif
    return 1;
  }
}

float _XRMiddleWare_GetBattery()  //获取电池电压
{
  float temp = xr_vol;
  return temp;
}

int _XRMiddleWare_GetWheelSpeed(
    u_int16_t *
        wheelspeed_buf_8)  //数据格式(list)[A轮转速，B轮转速，C轮转速，D轮转速，
                           //A轮目标转速，B轮目标转速，C轮目标转速，D轮目标转速]
{
  int rv = -1;
  for (size_t i = 0; i < 8; i++) {
    wheelspeed_buf_8[i] = xr_wheelspeed_list[i];
  }
  return 0;
}

int _XRMiddleWare_GetOdom(
    float *
        odom_buf_6)  //数据格式(list)[里程计X坐标，里程计Y坐标，里程计Yaw角度，机器人X轴线速度，机器人Y轴线速度，机器人Yaw轴角度]
{
  int rv = -1;
  for (size_t i = 0; i < 6; i++) {
    odom_buf_6[i] = xr_odom_list[i];
  }
  return 0;
}

int _XRMiddleWare_GetIMU(
    float *
        imu_buf_9)  //数据格式[陀螺仪Roll，陀螺仪Pitch，陀螺仪Yaw，线加速度X，线加速度Y，线加速度Z，姿态角Roll，姿态角Pitch，姿态角Yaw]
{
  int rv = -1;
  for (size_t i = 0; i < 9; i++) {
    imu_buf_9[i] = xr_imu_list[i];
  }
  return 0;
}

u_char _XR_readBytes(int fd) {
  int print_flag = 0;
  int rv = -1;
  u_char rev_byte[1];    //存储当前读取的1个字节
  u_char rev_buf[128];   //存储完整数据帧
  u_char rev_temp[128];  //存储临时接收所有整数
  u_int16_t count = 0;
  fd_set rset;
  int16_t temp16;
  int32_t temp32;
  float tempfloat;
  struct timeval timeout = {2, 0};
  memset(rev_buf, 0, sizeof(rev_buf));
  memset(rev_temp, 0, sizeof(rev_temp));
  while (1) {
    if (!Asyn) {
      /* code */
      if (!print_flag) {
        /* code */
        printf("Asyn is 0,set  _XRMiddleWare_init() \n");
        print_flag = 1;
      }
    } else {
      print_flag = 0;
      FD_ZERO(&rset);
      FD_SET(fd, &rset);
      timeout.tv_sec = 2;
      timeout.tv_usec = 2;
      rv = select(
          fd + 1, &rset, NULL, NULL, &timeout);  //使用select实现串口的多路通信
      if (rv < 0) {
        printf("select failure:%s\n", strerror(errno));
        close(fd);
      }
      if (rv == 0) {
        printf("select timeout\n");
        //  close(fd);
        continue;
      }

      memset(rev_byte, 0, sizeof(rev_byte));

      rv = read(fd, rev_byte, sizeof(rev_byte));
      if (rv < 0) {
        printf("read from buf failure:%s\n", strerror(errno));
        close(fd);
      }
      // printf("get %d \n", rev_byte[0]);
      rev_temp[count] = rev_byte[0];
      count++;

      if (count > 1) {
        if ((rev_temp[count - 2] == 0xaa) & (rev_temp[count - 1] == 0x55)) {
          count = 0;
          memset(rev_buf, 0, sizeof(rev_buf));
          memset(rev_temp, 0, sizeof(rev_temp));
        }
        if ((count > 3) & (rev_temp[count - 2] == 0xee) &
            (rev_temp[count - 1] == 0xee)) {
          memcpy(rev_buf, rev_temp, (count - 2) * sizeof(u_int8_t));

          // printf("\r\n");
          // for (size_t i = 0; i < count-2; i++)
          // {
          //     printf("%02x ",rev_buf[i]);
          // }
          // printf("\r\n");
          // printf("%d",count);
          if ((!check_crc(rev_buf, count - 2)) & (count == 0x38)) {
            // printf("got driverboard data\n");
            // type       1Bytes data[0]     0x01
            // ctl        1      data[1]     0x01
            // len        1      data[2]     0x32
            // imu_data   18     data[3:21]
            // dom_data   14     data[21:35]
            // wheelspeed 16     data[35:51]
            // vol        2      data[51:53]
            // tail       2      data[53:55] 0xff 0xff
            temp16 = rev_buf[51];
            temp16 = temp16 << 8 | rev_buf[52];
            xr_vol = (float)(temp16);
            xr_vol = xr_vol / 100;

            for (size_t i = 0; i < 3;
                 i++) {  //陀螺仪Roll，陀螺仪Pitch，陀螺仪Yaw， rad/s
              temp16 = rev_buf[3 + i * 2];
              temp16 = temp16 << 8 | rev_buf[4 + i * 2];
              xr_imu_list[i] = get_s16(temp16);
              xr_imu_list[i] = xr_imu_list[i] / 32.8 * 0.017453293;
            }
            for (size_t i = 0; i < 3;
                 i++) {  //线加速度X，线加速度Y，线加速度Z，
              temp16 = rev_buf[9 + i * 2];
              temp16 = temp16 << 8 | rev_buf[10 + i * 2];
              xr_imu_list[i + 3] = get_s16(temp16);
              xr_imu_list[i + 3] = xr_imu_list[i + 3] / 16384 * 9.8;
            }
            for (size_t i = 0; i < 3;
                 i++) {  //线加速度X，线加速度Y，线加速度Z，
              temp16 = rev_buf[15 + i * 2];
              temp16 = temp16 << 8 | rev_buf[16 + i * 2];
              xr_imu_list[i + 6] = get_s16(temp16);
            }

            for (size_t i = 0; i < 2; i++) {  //里程计  X Y
              temp32 = 0;
              temp32 = rev_buf[21 + i * 3] << 16 | rev_buf[22 + i * 3] << 8 |
                       rev_buf[23 + i * 3];
              xr_odom_list[i] = get_s24(temp32);
              xr_odom_list[i] = xr_odom_list[i] / 1000;
            }
            for (size_t i = 0; i < 4; i++) {  //里程计 YAW vx vy vyaw
              temp16 = rev_buf[27 + i * 2];
              temp16 = temp16 << 8 | rev_buf[28 + i * 2];
              xr_odom_list[i + 2] = get_s16(temp16);
              xr_odom_list[i + 2] = xr_odom_list[i + 2] / 1000;
            }
            //里程计  X Y交换，X=y Y=-x
            tempfloat = xr_odom_list[1];
            xr_odom_list[1] = -xr_odom_list[0];
            xr_odom_list[0] = tempfloat;
            //里程计  X Y交换，Vx = Vy   Vy = -Vx
            tempfloat = xr_odom_list[4];
            xr_odom_list[4] = -xr_odom_list[3];
            xr_odom_list[3] = tempfloat;

            for (size_t i = 0; i < 8; i++) {  // W1 W2 W3 W4 SW1 SW2 SW3 SW4
              temp16 = rev_buf[35 + i * 2];
              temp16 = temp16 << 8 | rev_buf[36 + i * 2];
              xr_wheelspeed_list[i] = get_s16(temp16);
            }
          }
          memset(rev_buf, 0, sizeof(rev_buf));
          memset(rev_temp, 0, sizeof(rev_temp));
          count = 0;
        }
      }
    }
  }
}

void Stop(int signo) {
  printf("捕获信号 %d，跳出...\n", signo);
  Asyn = 0;
  _exit(0);
}

void *cmdthread2(int fd) {
  int rv = -1;
  float imu_list
      [9];  //数据格式[陀螺仪Roll，陀螺仪Pitch，陀螺仪Yaw，线加速度X，线加速度Y，线加速度Z，姿态角Roll，姿态角Pitch，姿态角Yaw]
  float odom_list
      [6];  //数据格式(list)[里程计X坐标，里程计Y坐标，里程计Yaw角度，机器人X轴线速度，机器人Y轴线速度，机器人Yaw轴角度]
  u_int16_t
      wheelspeed_list[9];  //数据格式(list)[A轮转速，B轮转速，C轮转速，D轮转速，
                           //A轮目标转速，B轮目标转速，C轮目标转速，D轮目标转速]
  float vol;  //电池电压
  printf("cmd thread  stat ...\r\n");
  // rv = _XRMiddleWare_LowPower(fd);
  rv = _XRMiddleWare_init(fd);
  usleep(500000);
  rv = _XRMiddleWare_init(fd);
  usleep(500000);
  rv = _XRMiddleWare_SetWheelSpeed(fd, 0, 0, 0, 0);
  usleep(500000);
  while (1) {
    rv = _XRMiddleWare_SetWheelSpeed(fd, 100, 100, 100, 100);
    usleep(500000);
    rv = _XRMiddleWare_GetIMU(
        imu_list);  //数据格式[陀螺仪Roll，陀螺仪Pitch，陀螺仪Yaw，线加速度X，线加速度Y，线加速度Z，姿态角Roll，姿态角Pitch，姿态角Yaw]
    // usleep(20000);
    printf("\r\n");
    printf(
        "    Roll     Pitch    Yaw      x        y        z        mx       my "
        "      mz    :\r\n");
    for (size_t i = 0; i < 9; i++) {
      printf("%*.*f", 9, 3, imu_list[i]);
    }
    printf("\r\n");
  }
}

void *ansythread1(int fd) {
  int rv = -1;
  printf("Ansy thread  stat ...\r\n");
  _XR_readBytes(fd);
}

// int main(int argc, char **argv){
//    int fd = -1;
//    char *port = "/dev/xrbase";
//    int bps = 115200;
//
//    pthread_t Ansy_thread,Cmd_thread;
//    int rc = 0;
//    sleep(1);
//    ////////////////////test usart//////////////////////////////////
//    fd = open(port,O_RDWR|O_NOCTTY|O_NDELAY);//打开串口设备
//    if(fd < 0)
//        {
//            printf("open xrbase failure:%s\r\n",strerror(errno));
//            close(fd);
//            return -1;
//        }
//    printf("open com ok!\r\n");
//    set_serial(fd,bps,8,'N',1); // int set_serial(int fd,int nSpeed,int
//    nBits,char nEvent,int nStop) printf("open & set %s  %d
//    successfuly\r\n",port,bps);
//
//    tcflush(fd, TCIOFLUSH);//清空串口缓存
//
//    signal(SIGINT, Stop);
//
//    rc = pthread_create(&Ansy_thread,NULL,(void *)ansythread1,(void *)fd);
//    if (rc)
//    {
//        /* code */
//        printf("creat Ansy_thread error\n");
//        return 1;
//    }
//    rc = pthread_create(&Cmd_thread,NULL,(void *)cmdthread2,(void *)fd);
//    if (rc)
//    {
//        /* code */
//        printf("creat cmd_thread error\n");
//        return 1;
//    }
//
//    pthread_join(Ansy_thread,NULL);
//    pthread_join(Cmd_thread,NULL);
//    // close(fd);
//    // printf("close fd \r\n");
//    return 0 ;
//}
