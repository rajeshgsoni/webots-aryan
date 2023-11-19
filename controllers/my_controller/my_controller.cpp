// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include "LeaderRobot.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
    LeaderRobot leader;  // Create an instance of LeaderRobot
    int timeStep = leader.getBasicTimeStep();

    // Main control loop
    while (leader.step(timeStep) != -1) {
        leader.scanEnvironment();  // Scan for OOIs using LiDAR
        leader.identifyOOIs();     // Identify OOIs and their positions
        leader.commandScouts();    // Send commands to Scout Robots

        // Additional logic as required by your project
    }

    return 0;
}
