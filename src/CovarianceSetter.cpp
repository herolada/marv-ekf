// CovarianceSetter.cpp
// ROS Melodic node that rewrites pose/twist covariance diagonal of icp_odom messages
// and republishes them on icp_odom/cov.
//
// Parameters (private namespace):
//   ~cov_value (double, default: 0.001) – value to set on the six diagonal elements
//
// Build: add to your catkin package and list this file in add_executable()/target_link_libraries()
//        plus find_package(catkin COMPONENTS roscpp nav_msgs).
//
// Usage example:
//   rosrun your_pkg covariance_setter_node _cov_value:=0.05

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class CovarianceSetter
{
public:
  CovarianceSetter()
  {
    ros::NodeHandle nh;          // global namespace
    ros::NodeHandle pnh("~");   // private namespace for parameters

    // Load parameter once at startup (can be dynamic if you call pnh.getParam inside callback)
    pnh.param("cov_value", cov_value_, 0.001);

    pub_ = nh.advertise<nav_msgs::Odometry>("icp_odom/cov", 10);
    sub_ = nh.subscribe("icp_odom", 10, &CovarianceSetter::odomCallback, this);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    nav_msgs::Odometry out = *msg;   // copy incoming message

    // Overwrite diagonal entries of pose and twist covariance
    setDiagonal(out.pose.covariance);
    setDiagonal(out.twist.covariance);

    pub_.publish(out);
  }

  void setDiagonal(boost::array<double, 36>& cov)
  {
    // Indices of the diagonal elements in the 6×6 row‑major covariance matrix
    static const int diag_indices[6] = {0, 7, 14, 21, 28, 35};
    for (int idx : diag_indices)
    {
      cov[idx] = cov_value_;
    }
  }

  ros::Subscriber sub_;
  ros::Publisher  pub_;
  double cov_value_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "covariance_setter_node");
  CovarianceSetter node;
  ros::spin();
  return 0;
}
