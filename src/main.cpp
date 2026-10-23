#include "TankManager.hpp"

SerialTransport comms;

void setup()
{
    comms.init(&Serial, false);
}

void loop() {
    // Put your main code here, to run repeatedly:
    comms.updateFrame();
    if (comms.hasNewFrame()) {
        Frame f;
        comms.getFrame(f);

        digitalWrite(BUILTIN_LED, HIGH);
        
    }
}

