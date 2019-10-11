#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include "./cLogger/cLogger.h"
#include "./json/Json.h"


#include "lee.h"
#include "UsbToSrb.h"
#include "SrbMaster.h"
#include "Node.h"
#include "Broadcaster.h"
#include "./common_cluster/BaseCluster.h"
#include "./Nodes/MotorX2/NodeMotorX2.h"

using namespace srb::usb_bus;
using namespace srb;


NodeMotorX2 * key_ctrl_DUMOTOR=nullptr ;
UsbToSrb* mainbusUB=nullptr;
SrbMaster* mainSRBM=nullptr;



void chatterCallback(const geometry_msgs::Twist& msg)
{
  ROS_DEBUG("I heard: (%f,%f)", msg.linear.x, msg.angular.z);
  key_ctrl_DUMOTOR->Data()->ma.brake = no;
  key_ctrl_DUMOTOR->Data()->mb.brake = no;

  int speed_temp;
  speed_temp = 100*(msg.linear.x+msg.angular.z);
  if(speed_temp>10){speed_temp+=50;}
  if(speed_temp<-10){speed_temp-=50;}
  key_ctrl_DUMOTOR->Data()->ma.speed = speed_temp;

  speed_temp = 100*(msg.linear.x-msg.angular.z);
  if(speed_temp>10){speed_temp+=50;}
  if(speed_temp<-10){speed_temp-=50;}
  key_ctrl_DUMOTOR->Data()->mb.speed = speed_temp;

  key_ctrl_DUMOTOR->sendAccess(0);
  mainbusUB->doAccess();
}
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "SRB_test");
  ros::NodeHandle n;

  //FOLLOWING IS SRB CODING
  //To setting the SRB, you shold give name of the SRB bus and agv motor drive.
	std::string usb_port_name;
	std::string dumotor_name;
  if(!n.getParam("bus_name", usb_port_name)){
    ROS_ERROR("Bus name should set in bus_name.");
  }
  if(!n.getParam("feet_motor_name", dumotor_name)){
    ROS_ERROR("node name should set in bus_name.");
  }
  //Initialization a USB bus, and create a SrbMaster
  mainbusUB = new UsbToSrb();
  mainSRBM = new SrbMaster(mainbusUB);

  auto rev = mainbusUB->openUsbByName(usb_port_name.c_str());
	if (rev != done) {
		ROS_ERROR( "Try open port: [%s] fail!\n", usb_port_name.c_str());
		return 0;
	}
  ROS_INFO( "Open port: [%s] done!\n", usb_port_name.c_str());
	mainSRBM->scanNodes();

  Node * node  = mainSRBM->getNode(dumotor_name.c_str());
  if(node ==nullptr){
		ROS_ERROR( "Get node [%s] fail!\n", dumotor_name.c_str());return 0;
	}
	key_ctrl_DUMOTOR = static_cast<NodeMotorX2*>(node);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);



  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())  {

    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::spin();
  return 0;
}
