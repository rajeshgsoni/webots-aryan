#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>
#include "../BaseRobot/BaseRobot.hpp"


using namespace webots;

class ScoutRobot : public BaseRobot {
static constexpr int TIME_STEP = 64; // Adjust this value as needed for your simulation

public:
    ScoutRobot() {
        // Initialize sensors and actuators
        gps = getGPS("gps");
        gps->enable(TIME_STEP);
        outputGPSPosition();


    compass = getCompass("compass");
    compass->enable(TIME_STEP);

    camera = getCamera("camera");
    camera->recognitionEnable(TIME_STEP);
       
        
    }


bool checkForGreenOOI() {
    int recognizedObjects = camera->getRecognitionNumberOfObjects();
    return recognizedObjects > 0;  // Assuming green OOI returns 1 and red returns 0
}

    void run() {
    
            std::cout << "ScoutRobot run" << std::endl;
    //demonstrateMovement();  // Call the movement demonstration

        while (step(TIME_STEP) != -1) {
        auto message = receiveMessage();  // Capture the message from Leader
        
    
                
        if (!message.first.empty() && !message.second.empty()) {
            try {
                // Parse target coordinates
                double targetX = std::stod(message.first);
                double targetY = std::stod(message.second);
                
                targetPositionX = targetX;
                targetPositionY = targetY;
                std::cout << "Moving to target X: " << targetX << ", Y: " << targetY << std::endl;

                // Move to target, assuming a stop distance (adjust as needed)
                stopDistance = 1;  // Example stop distance
                if (moveToTarget(targetX, targetY, stopDistance)) {
                    std::cout << "Reached target." << std::endl;
                }
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error parsing target coordinates: " << e.what() << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "Target coordinates out of range: " << e.what() << std::endl;
            }
        }
        
        if (checkForGreenOOI()) {
            std::cout << "Green OOI detected." << std::endl;
              sendMessage("0", "10", "10");
              //break;
              //return ;
        }
        
        //outputGPSPosition();
        //moveToTarget(targetPositionX, targetPositionY, stopDistance);

        // Add additional tasks or methods to explore and report
        // exploreAndReport();
    }
    
    
    }


bool moveToTarget(double targetX, double targetY, double stopDistance) {
    // updateCurrentPosition();  // Assuming this function updates currentPositionX and currentPositionY
    
   std::cout << "ScoutRobot moveToTarget" << std::endl;
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
}

void moveTowards(double angle) {
    std::cout << "Inside moveTowards() with angle: " << angle << std::endl;    

    auto leftMotor = getMotor("left wheel motor");
    auto rightMotor = getMotor("right wheel motor");

    double baseSpeed = 5;  // Increase base speed
    double leftSpeed = baseSpeed;
    double rightSpeed = baseSpeed;

    if (angle > 0) {
        rightSpeed *= (1 - fabs(angle) / M_PI);  // Reduce right speed to turn left
    } else {
        leftSpeed *= (1 - fabs(angle) / M_PI);   // Reduce left speed to turn right
    }

    std::cout << "Setting velocity leftSpeed:" << leftSpeed << ", rightSpeed:" << rightSpeed << std::endl;    

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
}

private:
      webots::Camera *camera;

   void move(double speed) override {
        // Implementation of the move method specific to LeaderRobot
    }

    void rotate(double speed) override {
        // Implementation of the rotate method specific to LeaderRobot
    }  
    // Add other private members and methods as needed
};

int main(int argc, char **argv) {
    ScoutRobot scout;
    scout.run();
    return 0;
}
