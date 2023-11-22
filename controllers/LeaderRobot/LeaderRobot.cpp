#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include "../BaseRobot/BaseRobot.hpp"
#include <iostream>
#include <fstream>
#include <webots/Emitter.hpp>
#include <set>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Display.hpp>

using namespace webots;

class LeaderRobot : public BaseRobot {
    static constexpr int TIME_STEP = 64; // Adjust this value as needed for your simulation
  webots::Emitter *emitter;
    webots::Display *display;

public:

    LeaderRobot() {
        // Initialize sensors and actuators
        lidar = getLidar("lidar");
        lidar->enable(TIME_STEP);
        
        gps = getGPS("gps");
        gps->enable(TIME_STEP);
        outputGPSPosition();


    compass = getCompass("compass");
    compass->enable(TIME_STEP);

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
    
void logEvent(const std::string& message) {
    std::ofstream outputFile;
    outputFile.open("output.txt", std::ios::app); // Open in append mode

    if (outputFile.is_open()) {
        outputFile << message << std::endl; // Append message to file
        std::cout << message << std::endl;   // Also print to terminal
        outputFile.close();                  // Close the file
    } else {
        std::cerr << "Unable to open output.txt for logging." << std::endl;
    }
}
    

    void run() {
        
        lidar->enablePointCloud();

        std::cout << "LeaderRobot run method is called" << std::endl;
        bool running = true;
        
        while (step(TIME_STEP) != -1 && running) {
        //std::cout << "LeaderRobot run method is called" << std::endl;                
            running = scanEnvironmentAndDetectOOIs(); 
          //std::cout << "Running: " << running << std::endl;
          outputGPSPosition();
          
          if (!running)
          {
                  //outputGPSPosition();
          }
                   
        }



        // Implement other LeaderRobot specific behaviors here
    }
    
  void move(double speed) override {
        // Implementation of the move method specific to LeaderRobot
    }

    void rotate(double speed) override {
        // Implementation of the rotate method specific to LeaderRobot
    } 
    
    void stop() {
    std::cout << "Stopping all motors." << std::endl;

    auto frontLeftMotor = getMotor("front left wheel motor");
    auto frontRightMotor = getMotor("front right wheel motor");
    auto rearLeftMotor = getMotor("rear left wheel motor");
    auto rearRightMotor = getMotor("rear right wheel motor");

    frontLeftMotor->setVelocity(0);
    frontRightMotor->setVelocity(0);
    rearLeftMotor->setVelocity(0);
    rearRightMotor->setVelocity(0);

    std::cout << "All motors stopped." << std::endl;
}


private:
    Lidar *lidar;
    Motor *frontLeftMotor, *frontRightMotor, *rearLeftMotor, *rearRightMotor;
    std::set<std::pair<double, double>> dispatchedOOIs;  // To track dispatched OOIs
    //webots::GPS *gps;
    //webots::Compass *compass;
    //double currentPositionX;
    //double currentPositionY;
    double currentYaw;


   
    
bool scanEnvironmentAndDetectOOIs() {
    const float *lidarValues = lidar->getRangeImage();
    int numberOfPoints = lidar->getNumberOfPoints();
    double fieldOfView = lidar->getFov();
    double angleIncrement = fieldOfView / numberOfPoints;

    updateCurrentPosition();  // Update GPS and compass data

    for (int i = 0; i < numberOfPoints; ++i) {
        float range = lidarValues[i];
        if (range < 0.3) {  // Object within range
            double angle = -fieldOfView / 2 + i * angleIncrement;
            double relativeX = range * cos(angle);
            double relativeY = range * sin(angle);

            // Convert to absolute coordinates
            double absoluteX = currentPositionX + relativeX * cos(currentYaw) - relativeY * sin(currentYaw);
            double absoluteY = currentPositionY + relativeX * sin(currentYaw) + relativeY * cos(currentYaw);

            if (dispatchedOOIs.find({absoluteX, absoluteY}) == dispatchedOOIs.end()) {
                std::string scoutID = "1";  // Determine the appropriate Scout ID
                //sendMessage(scoutID, std::to_string(absoluteX), std::to_string(absoluteY));
              sendMessage(scoutID, std::to_string(currentPositionX), std::to_string(currentPositionY));

                dispatchedOOIs.insert({absoluteX, absoluteY});  // Mark as dispatched
                std::cout << "OOI at absolute position X: " << absoluteX << ", Y: " << absoluteY << " dispatched to Scout " << scoutID << std::endl;

                stop();
                return false;
            }
        }
    }
    return true;
}



};

int main(int argc, char **argv) {
    LeaderRobot leader;
    leader.run();
    return 0;
}
