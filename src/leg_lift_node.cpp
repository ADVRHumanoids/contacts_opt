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


    Eigen::Affine3d pose, pose1_8, pose2_8;
    
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
    
//     /* Pushing only */
//     ci.getPoseFromTf("ci/com", "ci/world_odom", pose);
//     pose.translation().z() -= 0.08;
//     Eigen::Vector3d com_ref = pose.translation();
//     ci.setTargetPose("com", pose, 2.0);
//     ci.waitReachCompleted("com");
//     
//     pose1_8.translation() << 0.55, 0.14, 0.1; // OK table close
//     pose1_8.linear() = Eigen::Quaterniond(0.19, 0.67, -0.04, 0.71).toRotationMatrix(); //w x y z
//     ci.setTargetPose("arm1_8", pose1_8, 4.0);
//     ci.waitReachCompleted("arm1_8");
// 	    
//     pose2_8.translation() << 0.55, -0.14, 0.1;
//     pose2_8.linear() = Eigen::Quaterniond(0.71, -0.04, 0.67, 0.19).toRotationMatrix();
//     ci.setTargetPose("arm2_8", pose2_8, 4.0);
//     ci.waitReachCompleted("arm2_8");   
//     
//     pose1_8.translation().x() += .30;
//     pose2_8.translation().x() += .30;
//     ci.setTargetPose("arm1_8", pose1_8, 5.0);
//     ci.setTargetPose("arm2_8", pose2_8, 5.0);
//     ci.waitReachCompleted("arm2_8");
     

    return 0;
}

