// File:          c_test.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/robot.hpp>
#include <webots/supervisor.hpp>
#include <stdio.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  // create the Robot instance.
  Supervisor *robot = new Supervisor();

  // get the time step of the current world.
  int timeStep = (int) robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  LED *led = robot->getLED("ledname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  
    // do this once only
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Burger");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
    printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  led->set(1);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
