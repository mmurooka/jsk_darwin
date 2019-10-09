#include <ros/ros.h>

#include "robotis_controller/robotis_controller.h"


using namespace robotis_framework;
using namespace dynamixel;

const double PROTOCOL_VERSION = 1.0;
const int SUB_CONTROLLER_ID = 200;
const int POWER_CTRL_TABLE = 24;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darwin_direct_controller");
  ros::NodeHandle nh;

  std::string device_name;
  nh.param<std::string>("device_name", device_name, "/dev/ttyUSB0");
  int baud_rate;
  nh.param<int>("baud_rate", baud_rate, 1000000);

  /**** [From Here] copied from op2_manager.cpp ****/
  // open port
  PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(device_name.c_str());
  bool ret = port_handler->setBaudRate(baud_rate);
  if (ret == false) {
    ROS_ERROR("Failed to set baudrate.");
  }

  // power on dxls
  PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  for (int count = 0; count < 5; count++) {
    int ret = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);
    if(ret != 0) {
      ROS_ERROR("Failed DXLs power on. [%s]", packet_handler->getRxPacketError(ret));
    } else {
      ROS_INFO("Succeeded in DXLs power on.");
      break;
    }
  }
  /**** [To Here] copied from op2_manager.cpp ****/

  return 0;
}
