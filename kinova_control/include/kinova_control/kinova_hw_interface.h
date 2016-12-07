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

#ifndef KINOVA_HW_INTERFACE_HPP
#define KINOVA_HW_INTERFACE_HPP

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

#include <kinova_driver/jaco_comm.h>

namespace kinova_control
{

class KinovaHWInterface: public ros_control_boilerplate::GenericHWInterface
{

public:
  /// \brief Constructor.
  ///
  /// \param nh Node handle for topics.
  /// \param jaco_comm - connection to arm
  KinovaHWInterface(ros::NodeHandle& nh, kinova::JacoComm& jaco_comm);

  /// \brief Read the state from the robot hardware.
  void read(ros::Duration &elapsed_time);

  /// \brief write the command to the robot hardware.
  void write(ros::Duration &elapsed_time);

  /**
   * Converts angles from DH convention to JACO's physical angles.
   *
   * @param angles Angles to be converted
   */
  void convertDHAnglesToPhysical(AngularInfo &angles);

  void convertPhysicalAnglesToDH(AngularInfo &angles);

  /**
   * Returns whether two vectors of doubles are similar enough
   *
   * @param goal The goal configuration.
   * @param current Current configuration.
   * @return true if they are close enough in value
   */
  bool posErrorWithinThreshold(std::vector<double> array1, std::vector<double> array2);

  /**
   * Returns whether two vectors of doubles are similar enough
   *
   * @param goal The goal configuration.
   * @param current Current configuration.
   * @return true if they are close enough in value
   */
  bool velErrorWithinThreshold(std::vector<double> array1, std::vector<double> array2);

  /**
   * Added by TC 3/19/15
   * Returns unwrapped angle
   *
   * @param newx: current angle
   * @param oldx: old angle
   * @return newx if abs(newx-oldx)<M_PI else add or subtract 2*M_PI from newx
   */
  double unwrap(double newx, double oldx);

  /**
   * \brief
   * \param input - description
   * \return true on success
   */
  double getShortestAngleDistance(double &reference, double &current);

  /**
   * Normalizes the angles to lie within -180 to 180 degrees.
   *
   * @param angles
   */
  void normalizeAngles(AngularInfo &angles);

  /**
   * Debug function
   *
   */
  void printAngles(const char* desc, AngularInfo &angles);

  /**
   * Normalizes any number to an arbitrary range by assuming the range wraps
   * around when going below min or above max.
   *
   * @param value The number to be normalized
   * @param start Lower threshold
   * @param end   Upper threshold
   * @return Returns the normalized number.
   */
  double normalize(const double value, const double start, const double end);

  /**
   * Returns true if the distance of the values lies within _tolerance_.
   *
   * @param first The first value
   * @param second The second value
   * @param tolerance Maximum distance of the values
   * @return TRUE if the distance is less than tolerance
   */
  bool areValuesClose(double first, double second, double tolerance);

  /** \breif Enforce limits for all values before writing */
  void enforceLimits(ros::Duration &period)
  {
    // TODO: implement this safety feature
  }

private:

  kinova::JacoComm&                            jaco_comm_;
  JoystickCommand                              joystick_command_;

  double                                       encoder_to_radian_ratio_;
  double                                       radian_to_encoder_ratio_;
  bool                                         convert_joint_velocities_;
  bool                                         actuators_enabled_;
  std::vector<double> old_joint_position_; //added by TC 9/19/15

};
}

#endif // KINOVA_HW_INTERFACE_HPP
