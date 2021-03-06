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

/* Author: Dave Coleman
   Desc:   ros_control main() entry point for controlling robots in ROS
*/

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <kinova_control/kinova_hw_interface.h>

// using namespace kinova_control;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_ros_control");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Load Jaco comm
  boost::recursive_mutex api_mutex;
  bool is_first_init = true;

  kinova::JacoComm* jaco_comm;
  while (ros::ok())
  {
    try
    {
      jaco_comm = new kinova::JacoComm(nh, api_mutex, is_first_init);
      break;
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM(e.what());
      kinova::JacoAPI api;
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      api.closeAPI();
      ros::Duration(1.0).sleep();
    }

    is_first_init = false;
  }

  // Initialize
  bool initialize = false;
  if (initialize)
  {
    jaco_comm->homeArm();
  }

  // Create the hardware interface specific to your robot
  boost::shared_ptr<kinova_control::KinovaHWInterface> hw_interface;
  hw_interface.reset(new kinova_control::KinovaHWInterface(nh, *jaco_comm));

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, hw_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
