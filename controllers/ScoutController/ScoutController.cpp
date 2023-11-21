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
                std::cout << "Moving to target X: " << targetX << ", Y: " << targetY << std::endl;

                // Move to target, assuming a stop distance (adjust as needed)
                double stopDistance = 0.5;  // Example stop distance
                if (moveToTarget(targetX, targetY, stopDistance)) {
                    std::cout << "Reached target." << std::endl;
                }
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error parsing target coordinates: " << e.what() << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "Target coordinates out of range: " << e.what() << std::endl;
            }
        }

        // Add additional tasks or methods to explore and report
        // exploreAndReport();
    }
    
    
    }


bool moveToTarget(double targetX, double targetY, double stopDistance) {
    // updateCurrentPosition();  // Assuming this function updates currentPositionX and currentPositionY
    
   std::cout << "ScoutRobot moveToTarget" << std::endl;

    double deltaX = targetX - currentPositionX;
    double deltaY = targetY - currentPositionY;
    //double distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY);

    /*
    if (distanceToTarget <= stopDistance) {
       std::cout << "Stopping movement" << std::endl;
        //stopMovement();  // Stops the robot
        return true;     // Target reached
    }
    */

    double angleToTarget = atan2(deltaY, deltaX);
    moveTowards(angleToTarget);
    std::cout << "Moving towards " << std::endl;    

    return false;  // Target not yet reached
}

void demonstrateMovement() {
    std::cout << "Demonstrating movement." << std::endl;

    // Simple forward movement for demonstration
    auto leftMotor = getMotor("left wheel motor");
    auto rightMotor = getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(5.0);  // Example speed
    rightMotor->setVelocity(5.0); // Example speed

    // Move for a certain number of steps
    for (int i = 0; i < 100; ++i) {
        if (step(TIME_STEP) == -1) {
            break;
        }
        //updateCurrentPosition();
        std::cout << "Current position: X = " << currentPositionX << ", Y = " << currentPositionY << std::endl;
    }

    // Stop the robot
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    std::cout << "Movement demonstration complete." << std::endl;
}


void moveTowards(double angle) {
    std::cout << "Inside moveTowards() with angle: " << angle << std::endl;    

    auto leftMotor = getMotor("left wheel motor");
    auto rightMotor = getMotor("right wheel motor");

    double baseSpeed = 5.0;  // Increase base speed
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
/*
void receiveCommandsFromLeader() {
    if (receiver->getQueueLength() > 0) {
        const char *data = (const char *)receiver->getData();
        std::string receivedData(data);
        receiver->nextPacket();

        std::istringstream iss(receivedData);
        std::string targetScoutID, targetX, targetY;
        if (std::getline(iss, targetScoutID, '|') &&
            std::getline(iss, targetX, '|') &&
            std::getline(iss, targetY, '|')) {

            if (targetScoutID == this->ID) {
                std::cout << "Received command for this Scout. X: " << targetX << ", Y: " << targetY << std::endl;
                // Process the command, move towards the target, etc.
            } else {
                std::cout << "Command for another Scout. Ignored." << std::endl;
            }
        }
    }
}
  */
  

  
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
