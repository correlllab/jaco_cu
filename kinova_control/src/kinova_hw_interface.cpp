/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:
*/

#include <kinova_control/kinova_hw_interface.h>
#include <kinova_driver/jaco_types.h>

namespace
{
/// \brief Convert Kinova-specific angle degree variations (0..180, 360-181) to
///        a more regular representation (0..180, -180..0).
inline void convertKinDeg(double& qd)
{
  static const double PI_180 = (M_PI / 180.0);

  // Angle velocities from the API are 0..180 for positive values,
  // and 360..181 for negative ones, in a kind of 2-complement setup.
  if (qd > 180.0) {
    qd -= 360.0;
  }
  qd *= PI_180;
}

inline void convertKinDeg(std::vector<double>& qds)
{
  for (int i = 0; i < qds.size(); ++i) {
    double& qd = qds[i];
    convertKinDeg(qd);
  }
}

inline void convertKinDeg(geometry_msgs::Vector3& qds)
{
  convertKinDeg(qds.x);
  convertKinDeg(qds.y);
  convertKinDeg(qds.z);
}
}

namespace kinova_control
{

KinovaHWInterface::KinovaHWInterface(ros::NodeHandle& nh,
				     kinova::JacoComm& jaco_comm)
  : ros_control_boilerplate::GenericHWInterface(nh)
  , jaco_comm_(jaco_comm)
  , convert_joint_velocities_(true)
  , actuators_enabled_(true)
{
  // Clear all previous trajectories and points
  jaco_comm_.initTrajectory();

  // Approximative conversion ratio from finger position (0..6000) to joint angle in radians (0..0.697).
  const static double FULLY_CLOSED = 6600;
  const static double FULLY_CLOSED_URDF = M_PI/180*40; //0.697;
  encoder_to_radian_ratio_ = FULLY_CLOSED_URDF / FULLY_CLOSED;
  radian_to_encoder_ratio_ = FULLY_CLOSED / FULLY_CLOSED_URDF;
  ROS_DEBUG_STREAM_NAMED("kinova_hw_interface","radian_to_encoder_ratio: " << radian_to_encoder_ratio_);
}

void KinovaHWInterface::read(ros::Duration &elapsed_time)
{
  kinova::FingerAngles fingers;
  jaco_comm_.getFingerPositions(fingers);

  // Query arm for current joint angles
  kinova::JacoAngles current_angles;
  jaco_comm_.getJointAngles(current_angles);
  kinova_msgs::JointAngles jaco_angles; // = current_angles.constructAnglesMsg();

  jaco_angles.joint1 = current_angles.Actuator1;
  jaco_angles.joint2 = current_angles.Actuator2;
  jaco_angles.joint3 = current_angles.Actuator3;
  jaco_angles.joint4 = current_angles.Actuator4;
  jaco_angles.joint5 = current_angles.Actuator5;
  jaco_angles.joint6 = current_angles.Actuator6;

  // Transform from Kinova DH algorithm to physical angles in radians, then place into vector array
  // J6 offset is 260 for Jaco R1 (type 0), and 270 for Mico and Jaco R2.
  double j6o = jaco_comm_.robotType() == 0 ? 260.0 : 270.0;
  joint_position_[0] = (180- jaco_angles.joint1) * (M_PI / 180);
  joint_position_[1] = (jaco_angles.joint2 - j6o) * (M_PI / 180);
  joint_position_[2] = (90-jaco_angles.joint3) * (M_PI / 180);
  joint_position_[3] = (180-jaco_angles.joint4) * (M_PI / 180);
  joint_position_[4] = (180-jaco_angles.joint5) * (M_PI / 180);
  joint_position_[5] = (j6o-jaco_angles.joint6) * (M_PI / 180);
  joint_position_[6] = encoder_to_radian_ratio_ * fingers.Finger1;
  joint_position_[7] = encoder_to_radian_ratio_ * fingers.Finger2;
  joint_position_[8] = encoder_to_radian_ratio_ * fingers.Finger3;

  // code block added by TC 3/19/15. Unwraps read joint angles
  if (old_joint_position_.size() == num_joints_) {
    joint_position_[0] = unwrap(joint_position_[0],old_joint_position_[0]);
    // Actuators 2 and 3 are not continuous (no unwrapping needed)
    joint_position_[3] = unwrap(joint_position_[3],old_joint_position_[3]);
    joint_position_[4] = unwrap(joint_position_[4],old_joint_position_[4]);
    joint_position_[5] = unwrap(joint_position_[5],old_joint_position_[5]);
  }
  old_joint_position_.resize(num_joints_);
  old_joint_position_[0] = joint_position_[0];
  old_joint_position_[3] = joint_position_[3];
  old_joint_position_[4] = joint_position_[4];
  old_joint_position_[5] = joint_position_[5];
  // finished code block

  // ROS_DEBUG_THROTTLE_NAMED(0.1, "raw_positions",
  //                          "%f %f %f %f %f %f %f %f %f",
  //                          joint_position_[0], joint_position_[1], joint_position_[2], joint_position_[3], joint_position_[4],
  //                          joint_position_[5], joint_position_[6], joint_position_[7], joint_position_[8]);

  // Joint velocities
  kinova::JacoAngles current_vels;
  kinova::FingerAngles current_fing_vels;
  jaco_comm_.getJointVelocities(current_vels, current_fing_vels);
  joint_velocity_[0] = current_vels.Actuator1;
  joint_velocity_[1] = current_vels.Actuator2;
  joint_velocity_[2] = current_vels.Actuator3;
  joint_velocity_[3] = current_vels.Actuator4;
  joint_velocity_[4] = current_vels.Actuator5;
  joint_velocity_[5] = current_vels.Actuator6;
  // No velocity for the fingers:
  joint_velocity_[6] = current_fing_vels.Finger1;
  joint_velocity_[7] = current_fing_vels.Finger2;
  joint_velocity_[8] = current_fing_vels.Finger3;


  if (false)
  ROS_DEBUG_THROTTLE_NAMED(0.1, "raw_velocities",
                           "%f %f %f %f %f %f %f %f %f",
                           joint_velocity_[0], joint_velocity_[1], joint_velocity_[2], joint_velocity_[3], joint_velocity_[4],
                           joint_velocity_[5], joint_velocity_[6], joint_velocity_[7], joint_velocity_[8]);

  if (convert_joint_velocities_) {
    convertKinDeg(joint_velocity_);
  }

  // Joint torques (effort)
  // NOTE: Currently invalid.
  kinova::JacoAngles joint_tqs;
  joint_effort_.resize(9);
  joint_effort_[0] = joint_tqs.Actuator1;
  joint_effort_[1] = joint_tqs.Actuator2;
  joint_effort_[2] = joint_tqs.Actuator3;
  joint_effort_[3] = joint_tqs.Actuator4;
  joint_effort_[4] = joint_tqs.Actuator5;
  joint_effort_[5] = joint_tqs.Actuator6;
  joint_effort_[6] = 0.0;
  joint_effort_[7] = 0.0;
  joint_effort_[8] = 0.0;

  // Copy current position over command vector, command points will
  // overwrite them for the controlled joints.
  joint_position_command_ = joint_position_;
  joint_velocity_command_ = joint_velocity_;

  // Check joystick status
  jaco_comm_.getJoystickValues(joystick_command_);
  if (joystick_command_.ButtonValue[2])
  {
    if (!actuators_enabled_)
    {
      std::cout << "Button 1 pressed " << std::endl;
      ROS_WARN_STREAM_NAMED("kinova_hw_interface","Arm enabled");
      jaco_comm_.enableActuators();
      actuators_enabled_ = true;
    }
  }
  if (joystick_command_.ButtonValue[3])
  {
    if (actuators_enabled_)
    {
      std::cout << "Button 2 pressed " << std::endl;
      ROS_WARN_STREAM_NAMED("kinova_hw_interface","Arm disabled");
      jaco_comm_.disableActuators();
      actuators_enabled_ = false;
    }
  }
  if (joystick_command_.ButtonValue[4])
  {
    std::cout << "Button 3 pressed " << std::endl;
    ROS_WARN_STREAM_NAMED("kinova_hw_interface","E-Stop Pressed");
    jaco_comm_.stopAPI();
    ros::Duration(1.0).sleep();
    ros::shutdown();
    exit(0);
  }

}

void KinovaHWInterface::write(ros::Duration &elapsed_time)
{
  if (false)
  ROS_DEBUG_THROTTLE_NAMED(0.1, "commanded_velocity",
                           "%f %f %f %f %f %f %f %f %f",
                           joint_velocity_command_[0],
                           joint_velocity_command_[1],
                           joint_velocity_command_[2],
                           joint_velocity_command_[3],
                           joint_velocity_command_[4],
                           joint_velocity_command_[5],
                           joint_velocity_command_[6],
                           joint_velocity_command_[7],
                           joint_velocity_command_[8]);
  // ROS_DEBUG_THROTTLE_NAMED(0.1, "commanded_position",
  //                          "%f %f %f %f %f %f %f %f %f",
  //                          joint_position_command_[0],
  //                          joint_position_command_[1],
  //                          joint_position_command_[2],
  //                          joint_position_command_[3],
  //                          joint_position_command_[4],
  //                          joint_position_command_[5],
  //                          joint_position_command_[6],
  //                          joint_position_command_[7],
  //                          joint_position_command_[8]);

  // ROS_DEBUG_THROTTLE_NAMED(0.1, "joint5",
  //                          "p_comm: %f v_comm: %f p: %f v: %f",
  //                          joint_position_command_[5],
  //                          joint_velocity_command_[5],
  //                          joint_position_[5],
  //                          joint_velocity_[5]);

  // Check if arm is running
  if (jaco_comm_.isStopped())
  {
    ROS_ERROR_STREAM("Could not send command because the arm is 'stopped'.");
    return;
  }

  // Decide if we actually need to send a command
  // if (velErrorWithinThreshold( joint_velocity_, joint_velocity_command_ ))
  // {
  //   ROS_DEBUG_THROTTLE_NAMED(0.1, "skip", "Not writing command because within threshold");
  //   return;
  // }

  // Create point
  TrajectoryPoint point;
  point.InitStruct();
  //memset(&point, 0, sizeof (point)); // TODO try removing this

  // The trajectory consists of angular velocity waypoints
  point.Position.Type = ANGULAR_VELOCITY;

  // Set up the trajectory point with waypoint values converted
  // from [rad/s] to [deg/s]
  point.Position.Actuators.Actuator1 = -joint_velocity_command_[0] * (180 / M_PI);
  point.Position.Actuators.Actuator2 = +joint_velocity_command_[1] * (180 / M_PI);
  point.Position.Actuators.Actuator3 = -joint_velocity_command_[2] * (180 / M_PI);
  point.Position.Actuators.Actuator4 = -joint_velocity_command_[3] * (180 / M_PI);
  point.Position.Actuators.Actuator5 = -joint_velocity_command_[4] * (180 / M_PI);
  point.Position.Actuators.Actuator6 = -joint_velocity_command_[5] * (180 / M_PI);

  //We set a negative velocity on every fingers to open them.
  point.Position.Fingers.Finger1 = joint_velocity_command_[6] * radian_to_encoder_ratio_; //(180 / M_PI);
  point.Position.Fingers.Finger2 = joint_velocity_command_[7] * radian_to_encoder_ratio_; //(180 / M_PI);
  point.Position.Fingers.Finger3 = joint_velocity_command_[8] * radian_to_encoder_ratio_; //(180 / M_PI);


  if (false)
  ROS_DEBUG_THROTTLE_NAMED(0.1, "commanded_velocity",
                           "%f %f %f %f %f %f %f %f %f",
                           joint_velocity_command_[0],
                           joint_velocity_command_[1],
                           joint_velocity_command_[2],
                           joint_velocity_command_[3],
                           joint_velocity_command_[4],
                           joint_velocity_command_[5],
                           joint_velocity_command_[6],
                           joint_velocity_command_[7],
                           joint_velocity_command_[8]);

  if (false)
  ROS_DEBUG_THROTTLE_NAMED(0.1, "finger_velocity",
                           "%f %f %f",
                           point.Position.Fingers.Finger1,
                           point.Position.Fingers.Finger2,
                           point.Position.Fingers.Finger3);

  jaco_comm_.addTrajectoryPoint(point);

}

void KinovaHWInterface::convertDHAnglesToPhysical(AngularInfo &angles)
{
  angles.Actuator1 = 180 - angles.Actuator1;
  angles.Actuator2 = 270 + angles.Actuator2;
  angles.Actuator3 =  90 - angles.Actuator3;
  angles.Actuator4 = 180 - angles.Actuator4;
  angles.Actuator5 = 180 - angles.Actuator5;
  angles.Actuator6 = 260 - angles.Actuator6;
}

/**
 * Converts angles from JACO's physical angles to DH convention.
 *
 * @param angles Angles to be converted
 */
void KinovaHWInterface::convertPhysicalAnglesToDH(AngularInfo &angles)
{
  angles.Actuator1 = 180 - angles.Actuator1;
  angles.Actuator2 = angles.Actuator2 - 270;
  angles.Actuator3 =  90 - angles.Actuator3;
  angles.Actuator4 = 180 - angles.Actuator4;
  angles.Actuator5 = 180 - angles.Actuator5;
  angles.Actuator6 = 260 - angles.Actuator6;
}

bool KinovaHWInterface::posErrorWithinThreshold(std::vector<double> array1, std::vector<double> array2)
{
  static const double ERROR_THRESHOLD = 0.00001;

  // Error check
  assert(array1.size() == array2.size());

  for (std::size_t i = 0; i < array1.size(); ++i)
  {
    if ( getShortestAngleDistance(array1[i], array2[i]) > ERROR_THRESHOLD )
      return false;
  }

  return true;
}

bool KinovaHWInterface::velErrorWithinThreshold(std::vector<double> array1, std::vector<double> array2)
{
  static const double ERROR_THRESHOLD = 0.0001;

  // Error check
  assert(array1.size() == array2.size());

  for (std::size_t i = 0; i < array1.size(); ++i)
  {
    if ( std::abs(array1[i] - array2[i]) > ERROR_THRESHOLD )
    {
      //ROS_DEBUG_STREAM_THROTTLE_NAMED(0.1, "error", "Joint " << i << " is beyond threshold with velocity error: " << std::abs(array1[i] - array2[i]));
      return false;
    }
  }

  return true;
}

double KinovaHWInterface::getShortestAngleDistance(double &reference, double &current)
{
  double error = reference - current;

  // Go the shortest way: if the error is bigger than PI, let's take
  // the other way around (typical e.g. for reference=-179 and feedback=179)
  // error in that case would be 358 but in fact those positions are
  // just 2 degrees from each other
  if (std::abs(error) > 180)
  {
    error = (error > 0 ? (reference - current) - 360 : 360 - (current - reference));
  }

  return error;
}


//Added by TC 3/19/15
//Unwraps angles for when >M_PI jumps occur between successive joint angles
double KinovaHWInterface::unwrap(double newx, double oldx)
{
  double difference = newx - oldx;
  double unwrappedx = newx;
  if (std::abs(difference) > M_PI)
  {
    unwrappedx = (difference > M_PI ? newx - (2*M_PI) : newx + (2*M_PI));
  }
  return unwrappedx;
}

void KinovaHWInterface::normalizeAngles(AngularInfo &angles)
{
  angles.Actuator1 = normalize(angles.Actuator1, -180.0, 180.0);
  angles.Actuator2 = normalize(angles.Actuator2, -180.0, 180.0);
  angles.Actuator3 = normalize(angles.Actuator3, -180.0, 180.0);
  angles.Actuator4 = normalize(angles.Actuator4, -180.0, 180.0);
  angles.Actuator5 = normalize(angles.Actuator5, -180.0, 180.0);
  angles.Actuator6 = normalize(angles.Actuator6, -180.0, 180.0);
}

void KinovaHWInterface::printAngles(const char* desc, AngularInfo &angles)
{
  char wp_info[1024];
  sprintf(wp_info, "%s:\t[%f, %f, %f, %f, %f, %f]",
          desc,
          angles.Actuator1,
          angles.Actuator2,
          angles.Actuator3,
          angles.Actuator4,
          angles.Actuator5,
          angles.Actuator6);
  ROS_INFO_STREAM(wp_info);
}

double KinovaHWInterface::normalize(const double value, const double start, const double end)
{
  const double width = end - start; //
  const double offsetValue = value - start; // value relative to 0

  return ( offsetValue - (floor(offsetValue / width) * width)) +start;
  // + start to reset back to start of original range
}

bool KinovaHWInterface::areValuesClose(double first, double second, double tolerance)
{
  return ((first <= second + tolerance) && (first >= second - tolerance));
}

} // namespace
