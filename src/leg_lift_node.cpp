#include <ros/ros.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/problem/Com.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/ros/RosImpl.h>


using namespace XBot;
using namespace XBot::Cartesian;



int main(int argc, char **argv)
{

    /* Init ROS node */
    ros::init(argc, argv, "leg_lift_node");
    ros::NodeHandle nh("leg_lift");
    ros::NodeHandle nh_priv("~");
   
    XBot::Cartesian::RosImpl ci;   


    Eigen::Affine3d pose;
    
    double com_y_offset = nh_priv.param("com_y", 0.05);
    double foot_z_lift = 0.1;
    
    ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
    pose.translation().x() = 0.0;
    pose.translation().y() = 0.0;
    pose.translation().z() -= 0.1;
    ci.setTargetPose("com", pose, 4.0);
    ci.waitReachCompleted("com");
    
    ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
    pose.translation().y() -= com_y_offset ;
    ci.setTargetPose("com", pose, 4.0);
    ci.waitReachCompleted("com");
    
    ci.getPoseFromTf("ci/wheel_1", "ci/world_odom", pose);
    pose.translation().z() += foot_z_lift;
    ci.setTargetPose("wheel_1", pose, 4.0);
    ci.waitReachCompleted("wheel_1");
    
    ci.getPoseFromTf("ci/wheel_1", "ci/world_odom", pose);
    pose.translation().z() -= foot_z_lift;
    ci.setTargetPose("wheel_1", pose, 4.0);
    ci.waitReachCompleted("wheel_1");
    
    ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
    pose.translation().y() += 2*com_y_offset ;
    ci.setTargetPose("com", pose, 4.0);
    ci.waitReachCompleted("com");
    
    ci.getPoseFromTf("ci/wheel_2", "ci/world_odom", pose);
    pose.translation().z() += foot_z_lift;
    ci.setTargetPose("wheel_2", pose, 4.0);
    ci.waitReachCompleted("wheel_2");
    
    ci.getPoseFromTf("ci/wheel_2", "ci/world_odom", pose);
    pose.translation().z() -= foot_z_lift;
    ci.setTargetPose("wheel_2", pose, 4.0);
    ci.waitReachCompleted("wheel_2");
    

    
    return 0;
}

