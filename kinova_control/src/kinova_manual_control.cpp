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
   Desc:   Allow joystick command of the arm
*/

#ifndef KINOVA_CONTROL__KINOVA_MANUAL_CONTROL
#define KINOVA_CONTROL__KINOVA_MANUAL_CONTROL

#include <ros_control_boilerplate/tools/joystick_manual_control.h>

namespace kinova_control
{

class KinovaManualControl : public ros_control_boilerplate::JoystickManualControl
{
public:

  /**
   * \brief Constructor
   */
  KinovaManualControl()
    : ros_control_boilerplate::JoystickManualControl("kinova_manual_control", "/jacob/kinova")
  {
    // Trajectory controllers
    trajectory_controllers_.push_back("velocity_trajectory_controller");
    trajectory_controllers_.push_back("ee_velocity_trajectory_controller");

    // Manual controllers
    manual_controllers_.push_back("velocity_controller_1");
    manual_controllers_.push_back("velocity_controller_2");
    manual_controllers_.push_back("velocity_controller_3");
    manual_controllers_.push_back("velocity_controller_4");
    manual_controllers_.push_back("velocity_controller_5");
    manual_controllers_.push_back("velocity_controller_6");
    manual_controllers_.push_back("ee_velocity_controller_1");
    manual_controllers_.push_back("ee_velocity_controller_2");
    manual_controllers_.push_back("ee_velocity_controller_3");

    // Load manual controllers
    loadManualControllers();
    
    ROS_INFO_STREAM_NAMED("kinova_manual_control","KinovaManualControl Ready.");
  }

  /**
   * \brief Response to joystick control
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Table of index number of /joy.buttons: ------------------------------------

    // 0 - A
    // 1 - B
    if (msg->buttons[1])
    {
      switchToManual();
    }
    // 2 - X
    if (msg->buttons[2])
    {
      switchToTrajectory();
    }
    // 3 - Y
    // 4 - LB
    // 5 - RB
    // 6 - back
    // 7 - start
    // 8 - power
    // 9 - Button stick left
    // 10 - Button stick right

    // Table of index number of /joy.axis: ------------------------------------

    // 0 - Left/Right Axis stick left
    // 1 - Up/Down Axis stick left
    // 2 - Left/Right Axis stick right
    // 3 - Up/Down Axis stick right
    // 4 - RT
    // 5 - LT
    // 6 - cross key left/right
    // 7 - cross key up/down
  }

private:

  
}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<KinovaManualControl> KinovaManualControlPtr;
typedef boost::shared_ptr<const KinovaManualControl> KinovaManualControlConstPtr;

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_manual_control");
  ROS_INFO_STREAM_NAMED("main", "Starting KinovaManualControl...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  kinova_control::KinovaManualControl server;

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}

#endif
