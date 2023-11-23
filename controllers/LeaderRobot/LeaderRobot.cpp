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
#include<webots/Keyboard.hpp>

using namespace webots;

class LeaderRobot : public BaseRobot {
    static constexpr int TIME_STEP = 64; // Adjust this value as needed for your simulation
  webots::Emitter *emitter;
    webots::Display *display;
    webots::Keyboard keyboard;

public:

    LeaderRobot() {
        // Initialize sensors and actuators
        lidar = getLidar("lidar");
        lidar->enable(TIME_STEP);
        
        gps = getGPS("gps");
        gps->enable(TIME_STEP);
        outputGPSPosition();
    keyboard.enable(TIME_STEP);


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
    
double calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}
    
 
bool moveToTarget(double stopDistance) {

    double distanceToTarget = calculateDistance(currentPositionX, currentPositionY, targetPositionX, targetPositionY);
    
    std::cout << "................moveToTarget distanceToTarget: " << distanceToTarget << " stopDistance: " << stopDistance  << std::endl; 

    if (distanceToTarget <= stopDistance) {
        move(0.0);
        rotate(0.0);
        return true;
    }

    double angleToTarget = atan2(targetPositionY - currentPositionY, targetPositionX - currentPositionX);
    double angleDifference = angleToTarget - currentYaw;

    while (angleDifference > M_PI) {
        angleDifference -= 2.0 * M_PI;
    }

    while (angleDifference <= -M_PI) {
        angleDifference += 2.0 * M_PI;
    }

    move(1.0);  // Adjust speed as needed
    rotate(angleDifference);  // Adjust rotation speed as needed

    return false;
 

    // updateCurrentPosition();  // Assuming this function updates currentPositionX and currentPositionY
    /*
   std::cout << "LeaderRobot moveToTarget" << std::endl;
    updateCurrentPosition();  // Update GPS and compass data

    double deltaX = targetX - currentPositionX;
    double deltaY = targetY - currentPositionY;
    double distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY);

    std::cout << "deltaX" << deltaX << ", deltaY " << deltaY << " distanceToTarget " << distanceToTarget << std::endl;    

    if (distanceToTarget <= stopDistance) {
       std::cout << "Stopping movement" << std::endl;
        //stopMovement();  // Stops the robot
        return true;     // Target reached
    }

    double angleToTarget = atan2(deltaY, deltaX);
    moveTowards(angleToTarget);
    std::cout << "Moving towards deltaX" << deltaX << ", deltaY " << deltaY << " angleToTarget " << angleToTarget << ", distanceToTarget " << distanceToTarget  << std::endl;    

    return false;  // Target not yet reached
    */
    
}

void moveTowards(double angle) {
    std::cout << "Inside moveTowards() with angle: " << angle << std::endl;    


        frontLeftMotor = getMotor("front left wheel motor");
        frontRightMotor = getMotor("front right wheel motor");
        rearLeftMotor = getMotor("rear left wheel motor");
        rearRightMotor = getMotor("rear right wheel motor");

        // Set the motors to velocity control mode
        frontLeftMotor->setPosition(INFINITY);
        frontRightMotor->setPosition(INFINITY);
        rearLeftMotor->setPosition(INFINITY);
        rearRightMotor->setPosition(INFINITY);


    double baseSpeed = 5;  // Increase base speed
    double leftSpeed = baseSpeed;
    double rightSpeed = baseSpeed;

    if (angle > 0) {
        rightSpeed *= (1 - fabs(angle) / M_PI);  // Reduce right speed to turn left
    } else {
        leftSpeed *= (1 - fabs(angle) / M_PI);   // Reduce left speed to turn right
    }

    std::cout << "Setting velocity leftSpeed:" << leftSpeed << ", rightSpeed:" << rightSpeed << std::endl;    

    frontLeftMotor->setVelocity(baseSpeed);
    frontRightMotor->setVelocity(baseSpeed);
        rearLeftMotor->setVelocity(baseSpeed);
    rearRightMotor->setVelocity(baseSpeed);

}

void keyboardControl()
{
        frontLeftMotor = getMotor("front left wheel motor");
        frontRightMotor = getMotor("front right wheel motor");

    //std::cout << "keyboardControl" << std::endl;   
        char const key = static_cast<char>(keyboard.getKey());
        std::cout << key << std::endl;
        switch (key) {
            case 'W':
                moveForwards(frontLeftMotor, frontRightMotor);
                std::cout << "W" << std::endl;   
                
                
                break;
            case 'A':
                turnLeft(frontLeftMotor, frontRightMotor);
                std::cout << "A" << std::endl;   
                
                break;
            case 'S':
                moveBackwards(frontLeftMotor, frontRightMotor);
                std::cout << "S" << std::endl;   
                
                break;
            case 'D':
                turnRight(frontLeftMotor, frontRightMotor);
                std::cout << "D" << std::endl;   
                
                break;
            case ' ':
                halt(frontLeftMotor, frontRightMotor);
                std::cout << "..." << std::endl;   
                
                break;
                
            default:
            //halt(&leftMotor, &rightMotor);
            break;
        }
     
}

    void run() {
        
        lidar->enablePointCloud();

        std::cout << "LeaderRobot run method is called" << std::endl;
        bool running = true;
        
        while (step(TIME_STEP) != -1) {
        
        keyboardControl();
       
        auto message = receiveMessage();  // Capture the message from Leader
        
        if (!message.first.empty() && !message.second.empty()) {
            try {
                // Parse target coordinates
                double targetX = std::stod(message.first);
                double targetY = std::stod(message.second);
                
                targetPositionX = targetX;
                targetPositionY = targetY;
                std::cout << "GREEN OOI target X: " << targetX << ", Y: " << targetY << std::endl;
                
                updateCurrentPosition();

                stopDistance = 1;  // Example stop distance
                if (moveToTarget(stopDistance)) {
                    std::cout << "Reached target." << std::endl;
                }
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error parsing target coordinates: " << e.what() << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "Target coordinates out of range: " << e.what() << std::endl;
            }
        }
        
        //std::cout << "LeaderRobot run method is called" << std::endl;  
        if(running){              
            //running = scanEnvironmentAndDetectOOIs(); 
            }
          //std::cout << "Running: " << running << std::endl;
          //outputGPSPosition();
          
         
                   
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


void moveForwards(webots::Motor* leftMotor, webots::Motor* rightMotor) {
    leftMotor->setVelocity(5.0);  // Set desired speed
    rightMotor->setVelocity(5.0); // Set desired speed
}

void moveBackwards(webots::Motor* leftMotor, webots::Motor* rightMotor) {
    leftMotor->setVelocity(-5.0);  // Set desired speed
    rightMotor->setVelocity(-5.0); // Set desired speed
}

void turnLeft(webots::Motor* leftMotor, webots::Motor* rightMotor) {
    leftMotor->setVelocity(-3.0);  // Set desired speed
    rightMotor->setVelocity(3.0); // Set desired speed
}

void turnRight(webots::Motor* leftMotor, webots::Motor* rightMotor) {
    leftMotor->setVelocity(3.0);  // Set desired speed
    rightMotor->setVelocity(-3.0); // Set desired speed
}

void halt(webots::Motor* leftMotor, webots::Motor* rightMotor) {
    leftMotor->setVelocity(0.0);  // Stop
    rightMotor->setVelocity(0.0); // Stop
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
        if (range < 1) {  // Object within range
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

