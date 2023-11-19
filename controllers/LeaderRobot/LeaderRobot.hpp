#include "../BaseRobot/BaseRobot.hpp"

class LeaderRobot : public BaseRobot {
public:
    LeaderRobot();
    virtual ~LeaderRobot();

    void run() override;
    // Add any additional functionalities specific to LeaderRobot
};
