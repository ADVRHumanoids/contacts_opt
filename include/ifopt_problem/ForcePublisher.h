#include <ros/ros.h>


namespace force_publisher {
    
  class ForcePublisher
  {
    
  public:
      
      ForcePublisher(std::vector<std::string> feet);
      
      void send_force(const Eigen::VectorXd& f_opt);
      void send_normal(const Eigen::VectorXd& n_opt);
      void send_wrench_manip(const Eigen::VectorXd& tau_manip);
      void send_force_arms(const Eigen::VectorXd& f_arms);
      
  private:
      
      std::vector<ros::Publisher> _pubs_force;
      std::vector<ros::Publisher> _pubs_force_arms;
      std::vector<ros::Publisher> _pubs_normal;
      ros::Publisher _pub_wrench_manip;
      
  };

  ForcePublisher::ForcePublisher(std::vector<std::string> feet)
  {
      ros::NodeHandle nh;
      
      
      for(auto l : feet)
      {
	  _pubs_force.push_back( nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/force_ref/" + l, 1) );
	  _pubs_normal.push_back( nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/normal/" + l, 1) );
      }
      
      _pub_wrench_manip = nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/wrench_manip/", 1);
      
      std::vector<std::string> arms  = {"arm1_8", "arm2_8"};
      
      for(auto i : arms)
      {
	  _pubs_force_arms.push_back( nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/force_arms/" + i, 1) );
      }
      
  }

  void ForcePublisher::send_force(const Eigen::VectorXd &f_opt)
  {
      
      for (int i : {0, 1, 2, 3}) 
      {
	  Eigen::Vector3d f =  f_opt.segment<3>(3*i);
	      
	  geometry_msgs::WrenchStamped msg;
	  msg.header.frame_id = "world";
	  msg.header.stamp = ros::Time::now();        
	  msg.wrench.force.x = f.x();
	  msg.wrench.force.y = f.y();
	  msg.wrench.force.z = f.z();
	
	_pubs_force[i].publish(msg);
	  
      }
      
  }

  void ForcePublisher::send_wrench_manip(const Eigen::VectorXd &tau_manip)
  {
      	      
	  geometry_msgs::WrenchStamped msg;
	  msg.header.frame_id = "world";
	  msg.header.stamp = ros::Time::now();        
	  msg.wrench.force.x = tau_manip[0];
	  msg.wrench.force.y = tau_manip[1];
	  msg.wrench.force.z = tau_manip[2];
	  msg.wrench.torque.x = tau_manip[3];
	  msg.wrench.torque.y = tau_manip[4];
	  msg.wrench.torque.z = tau_manip[5];
	
	_pub_wrench_manip.publish(msg);
	  
      
  }
  
  void ForcePublisher::send_normal(const Eigen::VectorXd &n_opt)
  {
      
      for (int i : {0, 1, 2, 3}) 
      {
	  Eigen::Vector3d n =  n_opt.segment<3>(3*i);
	      
	  geometry_msgs::WrenchStamped msg;
	  msg.header.frame_id = "world";
	  msg.header.stamp = ros::Time::now();        
	  msg.wrench.force.x = n.x();
	  msg.wrench.force.y = n.y();
	  msg.wrench.force.z = n.z();
	
	_pubs_normal[i].publish(msg);
	  
      }
      
  }
  
  void ForcePublisher::send_force_arms(const Eigen::VectorXd &f_arms)
  {
      
      for (int i : {0, 1}) 
      {     
	  
	  Eigen::Vector3d f =  f_arms.segment<3>(3*i);
	  
	  geometry_msgs::WrenchStamped msg;
	  msg.header.frame_id = "world";
	  msg.header.stamp = ros::Time::now();        
	  msg.wrench.force.x = f.x();
	  msg.wrench.force.y = f.y();
	  msg.wrench.force.z = f.z();
	
	_pubs_force_arms[i].publish(msg);
	  
      }
      
}

}

