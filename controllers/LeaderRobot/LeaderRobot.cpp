#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>
#include "../BaseRobot/BaseRobot.hpp"

using namespace webots;

class ScoutRobot : public Robot {
static constexpr int TIME_STEP = 64; // Adjust this value as needed for your simulation

public:
    ScoutRobot() {
        // Initialize sensors and actuators
    }
    
    void run() {
    

        while (step(TIME_STEP) != -1) {
        //std::cout << "LeaderRobot run method is called" << std::endl;                
           receiveCommandsFromLeader();
            exploreAndReport();
        }
        
    }

private:
  void receiveCommandsFromLeader() {
  
  //std::cout << "receiveCommandsFromLeader method is called" << std::endl;                
      // Example: Receiving data from Leader Robot
      auto receiver = getReceiver("receiver");
      receiver->enable(TIME_STEP);
  
      if (receiver->getQueueLength() > 0) {
          const char *data = (const char *)receiver->getData();
        // Example of processing the received data
        // The actual processing will depend on the format and content of your data
        std::string receivedData(data);
        std::cout << "Received data: " << receivedData << std::endl;
          
          // Process the received data
          receiver->nextPacket();
      }
  }
  
  void exploreAndReport() {
  
    std::cout << "exploreAndReport method is called" << std::endl;                

      // Example: Navigate and perform tasks, then report back
      // This could include moving towards a target, performing color detection, etc.
      // Actual implementation will depend on your robot's capabilities and tasks
  
      // Move towards a target - placeholder logic
      
   auto frontLeftMotor = getMotor("front left wheel motor");
    auto frontRightMotor = getMotor("front right wheel motor");
    auto rearLeftMotor = getMotor("rear left wheel motor");
    auto rearRightMotor = getMotor("rear right wheel motor");

    // Set the motors to velocity control mode
    frontLeftMotor->setPosition(INFINITY);
    frontRightMotor->setPosition(INFINITY);
    rearLeftMotor->setPosition(INFINITY);
    rearRightMotor->setPosition(INFINITY);

    // Set the velocity of the motors
    double leftSpeed = 6.28;   // Speed in radians per second
    double rightSpeed = 6.28;  // Speed in radians per second

    frontLeftMotor->setVelocity(leftSpeed);
    frontRightMotor->setVelocity(rightSpeed);
    rearLeftMotor->setVelocity(leftSpeed);
    rearRightMotor->setVelocity(rightSpeed);

    // Run for a certain number of simulation steps
    for (int i = 0; i < 100; ++i) {
        if (step(TIME_STEP) == -1) {
            break;
        }
    }

    // Stop the robot
    frontLeftMotor->setVelocity(0.0);
    frontRightMotor->setVelocity(0.0);
    rearLeftMotor->setVelocity(0.0);
    rearRightMotor->setVelocity(0.0);
    
         
    std::cout << "after setting position" << std::endl;                
     
  
      // Perform task at the target (e.g., color detection)
  
      // Report back to Leader
      // Similar to sending commands, use an Emitter to send data back
  }
  
void move(double speed) {
    // Implement movement logic for ScoutRobot
}

void rotate(double speed) {
    // Implement rotation logic for ScoutRobot
}


  
    // Add other private members and methods as needed
};

int main(int argc, char **argv) {
    ScoutRobot scout;
    scout.run();
    return 0;
}
