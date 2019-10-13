#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

#include "./cLogger/cLogger.h"
#include "./json/Json.h"
#include "lee.h"
#include "UsbToSrb.h"
#include "SrbMaster.h"
#include "Node.h"
#include "Broadcaster.h"
#include "./common_cluster/BaseCluster.h"
#include "./Nodes/MotorX2/NodeMotorX2.h"



#include <cmath>
#include <memory>


using namespace srb::usb_bus;
using namespace srb;

using float64=double;



int agvCmdToMotor(float64 linear,float64 angular){
    float64 speed_temp;
    speed_temp = 100*(linear+angular);
    if(speed_temp>10){speed_temp+=50;}
    if(speed_temp<-10){speed_temp-=50;}
    return round(speed_temp);
}
float64 Cmd_liner = 0;
float64 Cmd_angular = 0;
int Cmd_Upgrade = 0;

void cmdrelCallback(const geometry_msgs::Twist& msg)
{
    ROS_DEBUG("I heard: (%f,%f)", msg.linear.x, msg.angular.z);
    Cmd_liner = msg.linear.x;
    Cmd_angular = msg.angular.z;
    Cmd_Upgrade = 0;


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
    auto mainbusUB = std::make_unique<UsbToSrb>();
    auto mainSRBM = std::make_unique<SrbMaster>(mainbusUB.get());
    //mainbusUB = new UsbToSrb();
    //mainSRBM = new SrbMaster(mainbusUB);

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
    auto key_ctrl_DUMOTOR = static_cast<NodeMotorX2*>(node);



    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdrelCallback);
    tf::TransformBroadcaster tsbr;
    tf::Transform grond_to_agv;
    tf::Vector3 v;
    tf::Quaternion q;
    double angel=0;
    double PI_2 = 2*M_PI;
    v.setValue(0, 0, 0);


    int loop_rate = 100;
    ros::Rate loop(loop_rate);
    int count = 0;
    while (ros::ok())  {
        if(Cmd_Upgrade < 20){//for 200ms
            Cmd_Upgrade++;
        }
        else{
            Cmd_liner = Cmd_angular =0.0;
        }
        key_ctrl_DUMOTOR->Data()->ma.brake = no;
        key_ctrl_DUMOTOR->Data()->mb.brake = no;
        key_ctrl_DUMOTOR->Data()->ma.speed = agvCmdToMotor(Cmd_liner,Cmd_angular);
        key_ctrl_DUMOTOR->Data()->mb.speed = agvCmdToMotor(Cmd_liner,-Cmd_angular);
        key_ctrl_DUMOTOR->sendAccess(0);
        mainbusUB->doAccess();
        //access end


        //generate odom
        angel +=Cmd_angular/loop_rate;
        if (angel> PI_2){
            angel-=PI_2;
        }
        else if(angel<0){
            angel+=PI_2;
        }
        v += (tf::Vector3(Cmd_liner/loop_rate,0,0).rotate(tf::Vector3(0,0,1),angel));
        q.setRPY(0, 0, angel);

        grond_to_agv.setOrigin(v);
        grond_to_agv.setRotation(q);

        tsbr.sendTransform(tf::StampedTransform(grond_to_agv, ros::Time::now(),  "odom","edel"));






        ros::spinOnce();
        loop.sleep();
    }
    //ros::spin();
    return 0;
}
