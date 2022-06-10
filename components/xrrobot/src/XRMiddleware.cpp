// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <thread>

#include "xr_middleware.c"

class XRMiddleware {
 public:
  XRMiddleware(const char *p, int s) {
    port_ = p;
    speed_ = s;
  }

  int Init() {
    int ret = -1;
    fd_ = open(port_, O_RDWR | O_NOCTTY | O_NDELAY);  //打开串口设备
    if (fd_ < 0) {
      printf("open xrbase failure:%s\r\n", strerror(errno));
      return -1;
    }

    set_serial(fd_, speed_, 8, 'N', 1);
    printf("open port successfully: %s, fd: %d\n", port_, fd_);
    tcflush(fd_, TCIOFLUSH);  //  清空串口缓存
    //  usleep(20000);
    //  ret = _XRMiddleWare_deinit(fd_);
    //  usleep(20000);
    //  ret = _XRMiddleWare_clrOdom(fd_);
    //  usleep(20000);
    ret = _XRMiddleWare_init(fd_);
    usleep(20000);
    ret = _XRMiddleWare_init(fd_);
    usleep(20000);
    ansythread = std::make_shared<std::thread>([=]() { _XR_readBytes(fd_); });
    return ret;
  }

  int Deinit() {
    int ret = -1;
    ret = _XRMiddleWare_deinit(fd_);
    usleep(20000);
    ret = _XRMiddleWare_clrOdom(fd_);
    usleep(20000);
    ret = _XRMiddleWare_LowPower(fd_);
    usleep(3 * 1000 * 1000);
    return ret;
  }

  int SetPID(u_int16_t P, u_int16_t I, u_int16_t D) {
    return _XRMiddleWare_SetPID(fd_, P, I, D);
  }

  int SetParams(u_int16_t encoder_resolution,
                float wheel_radius,
                float wheel_track_div2,
                float half_long) {
    auto ret = _XRMiddleWare_set_encoder_resolution(fd_, encoder_resolution);
    usleep(20000);
    ret = _XRMiddleWare_set_wheel_diameter(fd_, wheel_radius);
    usleep(20000);
    ret = _XRMiddleWare_set_wheel_track(fd_, wheel_track_div2);
    usleep(20000);
    ret = _XRMiddleWare_set_half_long(fd_, half_long);
    return ret;
  }

  int SetVelocity(float x, float y, float yaw) {
    return _XRMiddleWare_SetVelocity(fd_, x, y, yaw);
  }

  int SetWheelspeed_(u_int16_t w1, u_int16_t w2, u_int16_t w3, u_int16_t w4) {
    return _XRMiddleWare_SetWheelSpeed(fd_, w1, w2, w3, w4);
  }

  int GetOdom(float *odom_buf_6) { return _XRMiddleWare_GetOdom(odom_buf_6); }

  int ClrOdom() { return _XRMiddleWare_clrOdom(fd_); }

  int GetIMU(float *imu_buf_9) { return _XRMiddleWare_GetIMU(imu_buf_9); }

  int GetWheelspeed_(u_int16_t *wheelspeed__buf_8) {
    return _XRMiddleWare_GetWheelSpeed(wheelspeed__buf_8);
  }

  float GetBattery() { return _XRMiddleWare_GetBattery(); }

  int LowPwr() { return _XRMiddleWare_LowPower(fd_); }

 private:
  int fd_ = -1;
  const char *port_ = nullptr;
  int speed_ = 115200;
  std::shared_ptr<std::thread> ansythread = nullptr;
};
