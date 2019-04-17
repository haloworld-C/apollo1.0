/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file PCIE_can_client.cpp
 * @brief the encapsulate call the api of PCIE can card according to can_client.h
 *interface
 **/

#include "modules/canbus/can_client/pcie/pcie_can_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#define bitrate0 "ip link set can0 type can bitrate 125000"
#define up0 "ifconfig can0 up"
#define down0 "ifconfig can0 down"
#define bitrate1 "ip link set can1 type can bitrate 125000"
#define up1 "ifconfig can1 up"
#define down1 "ifconfig can1 down"

namespace apollo {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool PCIECanClient::Init(const CANCardParameter &parameter) {
  /*
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }
  port_ = parameter.channel_id();
  */
  
  return true;
}

ErrorCode PCIECanClient::Start() {
  /*
  if (is_started_) {
    return ErrorCode::OK;
  }

  // open device
  // guss net is the device minor number, if one card is 0,1
  // if more than one card, when install driver u can specify the minior id
  // int32_t ret = canOpen(net, pCtx->mode, txbufsize, rxbufsize, 0, 0,
  // &dev_handler_);
  uint32_t mode = 0;
  // mode |= NTCAN_MODE_NO_RTR;
  int32_t ret = canOpen(port_ > 0 ? 1 : 0, mode, NTCAN_MAX_TX_QUEUESIZE,
                        NTCAN_MAX_RX_QUEUESIZE, 5, 5, &dev_handler_);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "open device error code [" << ret << "]: " << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // init config and state
  // After a CAN handle is created with canOpen() the CAN-ID filter is
  // cleared
  // (no CAN messages
  // will pass the filter). To receive a CAN message with a certain CAN-ID
  // or an
  // NTCAN-Event with
  // a certain Event-ID it is required to enable this ID in the handle
  // filter as
  // otherwise a
  // received  message or event is discarded by the driver for this handle.
  // 1. set receive message_id filter, ie white list
  int32_t id_count = 0x800;
  ret = canIdRegionAdd(dev_handler_, 0, &id_count);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "add receive msg id filter error code: " << ret << ", "
           << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // 2. set baudrate to 500k
  ret = canSetBaudrate(dev_handler_, NTCAN_BAUD_500);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "set baudrate error code: " << ret << ", " << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  */
  is_started_ = true;
  
  system(down0);
  system(bitrate0);
  system(up0);
  system(down1);
  system(bitrate1);
  system(up1);
  return ErrorCode::OK;
}

void PCIECanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    // int32_t ret = canClose(dev_handler_);
    /*
    if (ret != NTCAN_SUCCESS) {
      AERROR << "close error code:" << ret << ", " << GetErrorString(ret);
    } else {
      AINFO << "close PCIE can ok. port:" << port_;
    }
    */
    system(down0);
    system(down1);
  }
}

// Synchronous transmission of CAN messages
ErrorCode PCIECanClient::Send(const std::vector<CanFrame> &frames,
                             int32_t *const frame_num) {
  // 这是检查输入，所以不改
  //CHECK_NOTNULL(frame_num);
  //CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  // 这里要注意一下， is_started_ 是如何转变成 true 的状态的？
  // 这里先注释掉，因为用命令行启动 CAN 卡，肯定是启动状态的
  /*
  if (!is_started_) {
    AERROR << "PCIE can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  */
  int s, nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame;

  // 创建套接字
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(ifr.ifr_name, "can0" );

  // 指定 can0 设备
  ioctl(s, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // 将套接字与 can0 绑定
  bind(s, (struct sockaddr *)&addr, sizeof(addr));
  // 禁用过滤规则，本进程不接收报文，只负责发送
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  /*for (size_t i = 0; i < frames.size() && i < 8; ++i) {
    send_frames_[i].id = frames[i].can_id;
    send_frames_[i].len = frames[i].len;
    std::memcpy(send_frames_[i].data, frames[i].data, frames[i].len);
  }*/
 
  // Synchronous transmission of CAN messages
  // 同步传输CAN信息
  /*
  int32_t ret = canWrite(dev_handler_, send_frames_, frame_num, nullptr);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "send message failed, error code: " << ret << ", "
           << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  */
 for (int i = 0; i < sizeof(frame) && i < 8; ++i){
 
  nbytes = write(s, &frame[i], sizeof(frame[i]));
  if (nbytes != frame[i].len) {
    printf("Send Error frame[0]\n!");
            break; //发送错误，退出
  }
  sleep(1);
 }
 close(s);
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode PCIECanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) {
  
  // 老规矩，我们先不管是否启动，所以注释掉
  /*
  if (!is_started_) {
    AERROR << "PCIE can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }
  */



  // 检查输入是不是正常  MAX_CAN_RECV_FRAME_LEN初步设定为100
  /*if (*frame_num > 100 || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << 100
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }*/

  // 这个应该是PCIE_CAN 的函数，我们注释掉，自己写
  /*
  const int32_t ret = canRead(dev_handler_, recv_frames_, frame_num, nullptr);
  // rx timeout not log
  if (ret == NTCAN_RX_TIMEOUT) {
    return ErrorCode::OK;
  }
  if (ret != NTCAN_SUCCESS) {
    AERROR << "receive message failed, error code: " << ret << ", "
           << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }*/
  int s, nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame;
  struct can_filter rfilter[1];

  // 创建套接字
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(ifr.ifr_name, "can0" );
                                                                                        
  // 指定 can0 设备
  ioctl(s, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // 将套接字与 can0 绑定
  bind(s, (struct sockaddr *)&addr, sizeof(addr));

  // 定义接收规则，只接收表示符等于 0x11 的报文
  // 先注释掉，看看行不行
  // rfilter[0].can_id = 0x11;
  // rfilter[0].can_mask = CAN_SFF_MASK;

  // 设置过滤规则
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  while(1)
  {
    // 接收报文
    nbytes = read(s, &frame, sizeof(frame));
    // 显示报文
   /* if(nbytes > 0)
    {
      printf(“ID=0x%X DLC=%d data[0]=0x%X\n”, frame.can_id,
          frame.can_dlc, frame.data[0]);
    }*/
  }

  /*for (int32_t i = 0; i < *frame_num && i < 8; ++i) {
    CanFrame cf;
    cf.id = recv_frames_[i].can_id;
    cf.len = recv_frames_[i].len;
    std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].len);
    frames->push_back(cf);
  }*/
  close(s);

  return ErrorCode::OK;
}

/************************************************************************/
/************************************************************************/
/* Function: GetErrorString()                                            */
/* Return ASCII representation of NTCAN return code                     */
/************************************************************************/
/************************************************************************/
/*const int32_t ERROR_BUF_SIZE = 200;
std::string PCIECanClient::GetErrorString(const NTCAN_RESULT ntstatus) {
  struct ERR2STR {
    NTCAN_RESULT ntstatus;
    const char *str;
  };

  int8_t str_buf[ERROR_BUF_SIZE];

  static const struct ERR2STR err2str[] = {
      {NTCAN_SUCCESS, "NTCAN_SUCCESS"},
      {NTCAN_RX_TIMEOUT, "NTCAN_RX_TIMEOUT"},
      {NTCAN_TX_TIMEOUT, "NTCAN_TX_TIMEOUT"},
      {NTCAN_TX_ERROR, "NTCAN_TX_ERROR"},
      {NTCAN_CONTR_OFF_BUS, "NTCAN_CONTR_OFF_BUS"},
      {NTCAN_CONTR_BUSY, "NTCAN_CONTR_BUSY"},
      {NTCAN_CONTR_WARN, "NTCAN_CONTR_WARN"},
      {NTCAN_NO_ID_ENABLED, "NTCAN_NO_ID_ENABLED"},
      {NTCAN_ID_ALREADY_ENABLED, "NTCAN_ID_ALREADY_ENABLED"},
      {NTCAN_ID_NOT_ENABLED, "NTCAN_ID_NOT_ENABLED"},
      {NTCAN_INVALID_FIRMWARE, "NTCAN_INVALID_FIRMWARE"},
      {NTCAN_MESSAGE_LOST, "NTCAN_MESSAGE_LOST"},
      {NTCAN_INVALID_PARAMETER, "NTCAN_INVALID_PARAMETER"},
      {NTCAN_INVALID_HANDLE, "NTCAN_INVALID_HANDLE"},
      {NTCAN_NET_NOT_FOUND, "NTCAN_NET_NOT_FOUND"},
#ifdef NTCAN_IO_INCOMPLETE
      {NTCAN_IO_INCOMPLETE, "NTCAN_IO_INCOMPLETE"},
#endif
#ifdef NTCAN_IO_PENDING
      {NTCAN_IO_PENDING, "NTCAN_IO_PENDING"},
#endif
#ifdef NTCAN_INVALID_HARDWARE
      {NTCAN_INVALID_HARDWARE, "NTCAN_INVALID_HARDWARE"},
#endif
#ifdef NTCAN_PENDING_WRITE
      {NTCAN_PENDING_WRITE, "NTCAN_PENDING_WRITE"},
#endif
#ifdef NTCAN_PENDING_READ
      {NTCAN_PENDING_READ, "NTCAN_PENDING_READ"},
#endif
#ifdef NTCAN_INVALID_DRIVER
      {NTCAN_INVALID_DRIVER, "NTCAN_INVALID_DRIVER"},
#endif
#ifdef NTCAN_OPERATION_ABORTED
      {NTCAN_OPERATION_ABORTED, "NTCAN_OPERATION_ABORTED"},
#endif
#ifdef NTCAN_WRONG_DEVICE_STATE
      {NTCAN_WRONG_DEVICE_STATE, "NTCAN_WRONG_DEVICE_STATE"},
#endif
      {NTCAN_INSUFFICIENT_RESOURCES, "NTCAN_INSUFFICIENT_RESOURCES"},
#ifdef NTCAN_HANDLE_FORCED_CLOSE
      {NTCAN_HANDLE_FORCED_CLOSE, "NTCAN_HANDLE_FORCED_CLOSE"},
#endif
#ifdef NTCAN_NOT_IMPLEMENTED
      {NTCAN_NOT_IMPLEMENTED, "NTCAN_NOT_IMPLEMENTED"},
#endif
#ifdef NTCAN_NOT_SUPPORTED
      {NTCAN_NOT_SUPPORTED, "NTCAN_NOT_SUPPORTED"},
#endif
#ifdef NTCAN_SOCK_CONN_TIMEOUT
      {NTCAN_SOCK_CONN_TIMEOUT, "NTCAN_SOCK_CONN_TIMEOUT"},
#endif
#ifdef NTCAN_SOCK_CMD_TIMEOUT
      {NTCAN_SOCK_CMD_TIMEOUT, "NTCAN_SOCK_CMD_TIMEOUT"},
#endif
#ifdef NTCAN_SOCK_HOST_NOT_FOUND
      {NTCAN_SOCK_HOST_NOT_FOUND, "NTCAN_SOCK_HOST_NOT_FOUND"},
#endif
#ifdef NTCAN_CONTR_ERR_PASSIVE
      {NTCAN_CONTR_ERR_PASSIVE, "NTCAN_CONTR_ERR_PASSIVE"},
#endif
#ifdef NTCAN_ERROR_NO_BAUDRATE
      {NTCAN_ERROR_NO_BAUDRATE, "NTCAN_ERROR_NO_BAUDRATE"},
#endif
#ifdef NTCAN_ERROR_LOM
      {NTCAN_ERROR_LOM, "NTCAN_ERROR_LOM"},
#endif
      {(NTCAN_RESULT)0xffffffff, "NTCAN_UNKNOWN"} /* stop-mark 
  };

  const struct ERR2STR *es = err2str;

  do {
    if (es->ntstatus == ntstatus) {
      break;
    }
    es++;
  } while ((uint32_t)es->ntstatus != 0xffffffff);

#ifdef NTCAN_ERROR_FORMAT_LONG
  {
    NTCAN_RESULT res;
    char sz_error_text[60];

    res = canFormatError(ntstatus, NTCAN_ERROR_FORMAT_LONG, sz_error_text,
                         sizeof(sz_error_text) - 1);
    if (NTCAN_SUCCESS == res) {
      snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s - %s",
               es->str, sz_error_text);
    } else {
      snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
               es->str, ntstatus);
    }
  }
#else
  snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
           es->str, ntstatus);
#endif /* of NTCAN_ERROR_FORMAT_LONG 

  return std::string((const char *)(str_buf));
}*/

}  // namespace can
}  // namespace canbus
}  // namespace apollo
