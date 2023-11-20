#include "../BaseRobot/BaseRobot.hpp"
#include <webots/Lidar.hpp>

class LeaderRobot : public BaseRobot {
public:
    LeaderRobot();
    virtual ~LeaderRobot();

    void run() override;
    
private:
    webots::Lidar *lidar;
    void scanEnvironmentAndDetectOOIs();

    // Add any additional functionalities specific to LeaderRobot
};
