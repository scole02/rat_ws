
/* Author: Sam Cole
   Desc:   Rover Arm Turret hw interface
*/

#include <rat_control/rat_hw_interface.h>

namespace rat_ns
{
RatHWInterface::RatHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{

}

void RatHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  // // Resize vectors
  // joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO("RatHWInterface Ready.");
}

void RatHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
}

void RatHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);

}

void RatHWInterface::enforceLimits(ros::Duration& period)
{
  // // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
}


}  // namespace ros_ns
