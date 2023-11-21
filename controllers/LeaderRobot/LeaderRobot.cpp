#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include "../BaseRobot/BaseRobot.hpp"
#include <iostream>
#include <fstream>
#include <webots/Emitter.hpp>

using namespace webots;

class LeaderRobot : public BaseRobot {
    static constexpr int TIME_STEP = 64; // Adjust this value as needed for your simulation
  webots::Emitter *emitter;

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
        
        while (step(TIME_STEP) != -1) {
        //std::cout << "LeaderRobot run method is called" << std::endl;                
            scanEnvironmentAndDetectOOIs();        
        }



        // Implement other LeaderRobot specific behaviors here
    }
    
  void move(double speed) override {
        // Implementation of the move method specific to LeaderRobot
    }

    void rotate(double speed) override {
        // Implementation of the rotate method specific to LeaderRobot
    }  

private:
    Lidar *lidar;
    Motor *frontLeftMotor, *frontRightMotor, *rearLeftMotor, *rearRightMotor;


void scanEnvironmentAndDetectOOIs() {
    const float *lidarValues = lidar->getRangeImage();
    int numberOfPoints = lidar->getNumberOfPoints();
    double fieldOfView = lidar->getFov();
    double angleIncrement = fieldOfView / numberOfPoints;

    std::vector<std::pair<double, double>> points;
    for (int i = 0; i < numberOfPoints; ++i) {
        float range = lidarValues[i];
        if (range < 0.3) {
            double angle = -fieldOfView / 2 + i * angleIncrement;
            points.push_back({range * cos(angle), range * sin(angle)});
        }
    }

    // Basic clustering
    std::vector<std::pair<double, double>> oois;
    double clusterThreshold = 10; // Threshold to consider points in the same cluster
    for (std::vector<std::pair<double, double>>::size_type i = 0; i < points.size(); ++i) {
        if (i == 0 || (sqrt(pow(points[i].first - points[i-1].first, 2) + 
                          pow(points[i].second - points[i-1].second, 2)) > clusterThreshold)) {
            oois.push_back(points[i]);
        }
    }

    // Log OOIs
    for (const auto& ooi : oois) {
    
        //logEvent("OOI discovered at x:" + std::to_string(ooi.first) + " y:" + std::to_string(ooi.second));

        // Example of sending a message to a Scout Robot
        // You'll replace 'targetX', 'targetY', and 'scoutID' with actual values
        double targetX = ooi.first; // X coordinate of target
        double targetY = ooi.second; // Y coordinate of target
        std::string scoutID = "1"; // ID of the Scout Robot to send the message to

        sendMessage(scoutID, std::to_string(targetX), std::to_string(targetY));
        
    }
}




};

int main(int argc, char **argv) {
    LeaderRobot leader;
    leader.run();
    return 0;
}
