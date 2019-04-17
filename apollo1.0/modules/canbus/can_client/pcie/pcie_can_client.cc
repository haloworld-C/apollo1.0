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
//#define down1 "ifconfig can1 down"
namespace apollo {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool PCIECanClient::Init(const CANCardParameter &parameter) {
	 return true;
}

ErrorCode PCIECanClient::Start() {
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
    system(down0);
    system(down1);
  }
}


ErrorCode PCIECanClient::Send(const std::vector<CanFrame>& frames,
                                int32_t *const frame_num) {
  int s, nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  //struct CanFrame frame;

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
 for (int i = 0; i < 8; ++i)
 {
 
  nbytes = write(s, &frames[i], sizeof(frames[i]));
  if (nbytes != frames[i].len) 
  {
    printf("Send Error frame[0]\n!");
            break; //发送错误，退出
  }
  sleep(1);
 }
 close(s);
  return ErrorCode::OK;
}
ErrorCode PCIECanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) 
{
  int s;
  // int nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  //struct CanFrame frame;
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
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  
  for(int i = 0; i < 8; i++)
	{
	  CanFrame cf;
    // nbytes = read(s, &frames, sizeof(frames)); //接收报文
	        //显示报文
	 /*   if(nbytes > 0)
	    {
	        printf("ID=0x%X DLC=%d data[0]=0x%X\n", frames[i]->id,
	                frames->len, frames->data[i]);
	    }*/
	}
 // while(1)
  //{
    // 接收报文
    //nbytes = read(s, &frames, sizeof(frames));
    // 显示报文
  close(s);

  return ErrorCode::OK;
}
}
}
}