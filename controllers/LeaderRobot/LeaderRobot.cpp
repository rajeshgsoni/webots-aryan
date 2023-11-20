#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include "../BaseRobot/BaseRobot.hpp"
#include <iostream>

using namespace webots;

class LeaderRobot : public Robot {
    static constexpr int TIME_STEP = 64; // Adjust this value as needed for your simulation

public:
    LeaderRobot() {
        // Initialize sensors and actuators
        lidar = getLidar("lidar");
        lidar->enable(TIME_STEP);

        frontLeftMotor = getMotor("front left wheel motor");
        frontRightMotor = getMotor("front right wheel motor");
        rearLeftMotor = getMotor("rear left wheel motor");
        rearRightMotor = getMotor("rear right wheel motor");

        // Set the motors to velocity control mode
        frontLeftMotor->setPosition(INFINITY);
        frontRightMotor->setPosition(INFINITY);
        rearLeftMotor->setPosition(INFINITY);
        rearRightMotor->setPosition(INFINITY);
    }

    void run() {
    
lidar->enablePointCloud();

        std::cout << "LeaderRobot run method is called" << std::endl;
        
                while (step(TIME_STEP) != -1) {
        //std::cout << "LeaderRobot run method is called" << std::endl;                
        scanEnvironmentAndDetectOOIs();        
        }



        // Implement other LeaderRobot specific behaviors here
    }

private:
    Lidar *lidar;
    Motor *frontLeftMotor, *frontRightMotor, *rearLeftMotor, *rearRightMotor;

    void scanEnvironmentAndDetectOOIs() {
    const float *lidarValues = lidar->getRangeImage();
    int numberOfPoints = lidar->getNumberOfPoints();
    

   //std::cout << "numberOfPoints: " << numberOfPoints << std::endl;

    // Process LiDAR data to detect OOIs
    for (int i = 0; i < numberOfPoints; ++i) {
        float range = lidarValues[i];
        
        if (range < 0.5) {
            // Object detected, process accordingly
            std::cout << "Object detected. Range: " << range << std::endl;

        }

        // Analyze range data to find OOIs
    }

        // Once OOIs are detected, handle their data (e.g., store positions)
    }

};

int main(int argc, char **argv) {
    LeaderRobot leader;
    leader.run();
    return 0;
}
