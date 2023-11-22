// File:          BaseRobot.hpp
// Date:          XX/XX/XXXX
// Description:   Header file of BaseRobot to be inherited by the Leader and Scout robots classes.
// Author:        XXX XXX
// zID:           z1234567
// Modifications:

#pragma once

#include <iostream>
#include <memory>
#include <sstream>
#include <cstring>
#include <string>

#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

constexpr int TIME_STEP{ 64 };

class BaseRobot : public webots::Robot {
public:
    BaseRobot();
    virtual ~BaseRobot();

    virtual void run() = 0;
    virtual void move(double speed) = 0;
    virtual void rotate(double speed) = 0;


    void keyboardControl();
    void updateCurrentPosition();
    void outputGPSPosition();
    void setTargetPosition(double x, double y);
    bool moveToTarget(double stopDistance);

    void sendMessage(const std::string& ID, const std::string& data0, const std::string& data1);
    std::pair<std::string, std::string> receiveMessage();

protected:
    std::string ID{};
    webots::GPS *gps;
    webots::Compass *compass;
    
    double currentPositionX{};
    double currentPositionY{};
    double currentYaw{};
    double targetPositionX{};
    double targetPositionY{};
    double stopDistance{};    

private:
    std::unique_ptr<webots::Receiver> receiver{};
    std::unique_ptr<webots::Emitter> emitter{};
};
