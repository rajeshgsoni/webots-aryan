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

        while (step(TIME_STEP) != -1) {
            //receiveCommandsFromLeader();
            receiveMessage();
            //exploreAndReport();
        }
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
  
  void exploreAndReport() {
      // Example: Navigate and perform tasks, then report back
      // This could include moving towards a target, performing color detection, etc.
      // Actual implementation will depend on your robot's capabilities and tasks
  
      // Move towards a target - placeholder logic
      auto leftMotor = getMotor("front left wheel motor");
      auto rightMotor = getMotor("front right wheel motor");
      leftMotor->setPosition(10.0);   // Example values
      rightMotor->setPosition(10.0);  // Example values
  
      // Perform task at the target (e.g., color detection)
  
      // Report back to Leader
      // Similar to sending commands, use an Emitter to send data back
  }
  
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
