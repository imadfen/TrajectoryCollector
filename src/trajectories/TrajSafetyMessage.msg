// 1. Include C++ Definitions
cplusplus {{
#include "veins/modules/messages/DemoSafetyMessage_m.h"
}}

// 2. Use the 'veins' namespace to match the parent class
namespace veins;

// 3. Forward Declare the parent class
class DemoSafetyMessage;

// 4. Define the Packet
packet TrajSafetyMessage extends DemoSafetyMessage {
    unsigned long sequenceNumber;
    int senderId;
}