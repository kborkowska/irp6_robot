#ifndef ForceTransformation_H_
#define ForceTransformation_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Wrench.h"
#include "kdl_conversions/kdl_msg.h"
#include <Eigen/Dense>

class ForceTransformation : public RTT::TaskContext {
 public:
  ForceTransformation(const std::string& name);
  virtual ~ForceTransformation();

  virtual bool configureHook();
  virtual bool startHook();
  void updateHook();

 private:

  RTT::InputPort<geometry_msgs::Pose> port_current_wrist_pose_;

  RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_wrench_;
  RTT::OutputPort<geometry_msgs::Wrench> port_output_wrist_wrench_;
  RTT::OutputPort<geometry_msgs::Wrench> port_output_end_effector_wrench_;
  RTT::InputPort<geometry_msgs::Pose> port_tool_;

  KDL::Wrench force_offset_;

  // ForceTrans
  double tool_weight_;

  KDL::Wrench gravity_force_torque_in_base_;
  KDL::Wrench reaction_force_torque_in_wrist_;
  KDL::Vector gravity_arm_in_wrist_;
  KDL::Frame tool_mass_center_translation_;

  geometry_msgs::Pose sensor_frame_property_;
  bool is_right_turn_frame_property_;

  KDL::Frame sensor_frame_kdl_;

};

#endif /* ForceTransformation_H_ */
