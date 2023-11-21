// File:          BaseRobot.cpp
// Date:          XX/XX/XXXX
// Description:   Implementation of BaseRobot to be inherited by the Leader and Scout robots classes.
// Author:        XXX XXX
// zID:           z1234567
// Modifications:

#include "BaseRobot.hpp"

BaseRobot::BaseRobot()
    : ID{ getName() }
    , receiver{ getReceiver("receiver") }
    , emitter{ getEmitter("emitter") } {
    receiver->enable(TIME_STEP);
}


BaseRobot::~BaseRobot(){

}


void BaseRobot::sendMessage(const std::string& ID, const std::string& data0, const std::string& data1) {
    std::cout << "Sending message to " << ID << std::endl;
    std::string message{};
    message.append(ID);
    message.append("|");
    message.append(data0);
    message.append("|");
    message.append(data1);
    emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
}

std::pair<std::string, std::string> BaseRobot::receiveMessage() {
    //std::cout << "Checking receiver queue..." << std::endl;
    if (receiver->getQueueLength() > 0) {
        //std::cout << "Message received. Queue length: " << receiver->getQueueLength() << std::endl;

        std::string message{ static_cast<const char*>(receiver->getData()) };
        //std::cout << "Raw message: " << message << std::endl;

        receiver->nextPacket();

        std::istringstream iss{ message };
        std::string incomingId{};
        std::getline(iss, incomingId, '|');

        std::cout << "Incoming ID: " << incomingId << std::endl;
        if (ID.compare(incomingId) == 0) {
            std::cout << "ID matches. Processing message..." << std::endl;
            std::string data0{};
            std::string data1{};
            if (std::getline(iss, data0, '|') && std::getline(iss, data1, '|')) {
                std::cout << "Received message with matching ID: " << message << std::endl;
                return std::make_pair(data0, data1);
            }
        } else {
            std::cout << "Msg for ID # " << incomingId << " does not match my ID # " << ID << "! Ignoring message..." << std::endl;
        }
    } else {
        //std::cout << "No messages in receiver queue." << std::endl;
    }
    return std::make_pair("", "");
}


// Implementations for virtual functions
void BaseRobot::run() {
    // Placeholder implementation - Override in derived classes
}

void BaseRobot::move(double speed) {
    // Placeholder implementation - Override in derived classes
}

void BaseRobot::rotate(double speed) {
    // Placeholder implementation - Override in derived classes
}
