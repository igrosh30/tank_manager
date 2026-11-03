#include "WaterTank_Worker.hpp"


//TODO's
/*
    1- add timeouts so even if the sensors are not working, we can still perform this! ✅
    2- add the restart / reboot safety state  
    3- add a test function for the GPIOS before run the tank state update
    4- add a fuction that looks to diagnose the GPIOS everytime! maybe it can start working?... or still if we have this it can send us the error! 
    5- Check if we need to be calling always pumpOn for pumping the water- I thikn not bc this is a relay - check 

    On the field:
    **Bc of the delay: check where the water level will get from the command to stopPump and actually stoping! 

*/

void WaterTank_Worker::init(Stream* port)//WaterTank
{
    //----------COM INITIALISTION------------//
    #ifndef DEBUG_SERIAL
    if(port == nullptr) return;
    jetson_link.init(port,false);
    #endif

    //----------PIN INITIALISATION-----------//
    pinMode(SENSOR_UP, INPUT);
    //pinMode(SENSOR_DOWN,INPUT);
    pinMode(PUMP_FASE1,OUTPUT);
    pinMode(PUMP_FASE2,OUTPUT);
    pinMode(VALVE,OUTPUT);

    //stability:
    // this->pumpOFF();
    // this->close_valve();
    set_next_state(tank_state::CHECK_STABILITY, "set init state");//NEEDS to receive a PING! 

}

//-------------------------------------------------------------------------------
//-------------------------GPIO FUNCTIONS----------------------------------------
//-------------------------------------------------------------------------------
void WaterTank_Worker::read_sensors()
{
    //sensor_down_val=digitalRead(SENSOR_DOWN);
    sensor_up_val = digitalRead(SENSOR_UP);
}

bool WaterTank_Worker::isEmpty() const
{
    return sensor_up_val == LOW;
}
bool WaterTank_Worker::isTopEmpty() const {
    return sensor_up_val == LOW;
}
bool WaterTank_Worker::isFull() const {
    return sensor_up_val == HIGH;
}

void WaterTank_Worker::set_actuator(int gpio, int val)
{
    digitalWrite(gpio,val);
}

void WaterTank_Worker::close_valve()
{
    this->set_actuator(VALVE,LOW);//check if 0 is the val to turn off the valve! 
}

void WaterTank_Worker::open_valve()
{
    this->set_actuator(VALVE,HIGH);//check if 1 is the val to turn off the valve! 

}

void WaterTank_Worker::pumpON()
{
    this->set_actuator(PUMP_FASE1,HIGH);//check if 0 is the val to turn off the valve! 
    this->set_actuator(PUMP_FASE2,HIGH);//check if 0 is the val to turn off the valve! 
}
void WaterTank_Worker::pumpOFF()
{
    this->set_actuator(PUMP_FASE1,LOW);
    this->set_actuator(PUMP_FASE2,LOW);
}

//-------------------------------------------------------------------------------
//------------------------F.State Machines---------------------------------------
//-------------------------------------------------------------------------------
void WaterTank_Worker::update_state()
{
    read_sensors();
    switch (this->current_tank_state)
    {
        case tank_state::CHECK_STABILITY:
            if (isFull()) {
                open_valve();
                set_next_state(tank_state::DRAIN, "boot: tank FULL");
            }
            else if (isEmpty()) {
                pumpON();
                set_next_state(tank_state::FILL, "boot: tank EMPTY");
            }
            break;

        case tank_state::READY:
            if (startCycleFlag)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                startCycleFlag = false;
                open_valve();
                set_next_state(tank_state::DRAIN, "START_CYCLE command");
            }
            /*
            THIS TIMEOUT HELPS IF THE COM ISn't workin! 
            else if (time_in_state() >= SENSOR_TIMEOUT_MS) {
                open_valve();
                set_next_state(tank_state::DRAIN, "autonomous timer");}*/
            break;

        case tank_state::DRAIN://stays here at least 20s
            /*
            we'll add for now only a timeout! 
            
            if(time_in_state() < 20000)//20s needs to drain
            {
                sensor_up_val = HIGH;
                return;
            }
            if (isEmpty() || time_in_state() >= DRAIN_TIMEOUT_MS)
            {
                close_valve();
                pumpON();
                set_next_state(tank_state::FILL, "bottom sensor: empty");
            }*/
           if (time_in_state() >= DRAIN_TIMEOUT_MS)
            {
                close_valve();
                pumpON();
                set_next_state(tank_state::FILL, "bottom sensor: empty");
            }
            break;

        case tank_state::FILL:
            if (isFull() ||time_in_state() >= FILL_TIMEOUT_MS )
            {
                pumpOFF();
                digitalWrite(LED_BUILTIN, LOW);
                set_next_state(tank_state::READY, "top sensor: full");
            }
            break;
    }
}

void WaterTank_Worker::set_next_state(tank_state s, const char* reason)
{
    if (s != current_tank_state)
    {
#if DEBUG_SERIAL
        Serial.print(millis());
        Serial.print(F("ms  "));
        Serial.print(state_name(current_tank_state));
        Serial.print(F("  ->  "));
        Serial.print(state_name(s));
        Serial.print(F("   ["));
        Serial.print(reason);
        Serial.println(F("]"));
#endif
        current_tank_state = s;
        state_entry = millis();
    }
}
unsigned long WaterTank_Worker::time_in_state() const {
    return millis() - state_entry;
}




//-------------------------------------------------------------------------------
//--------------------------COM link With JETSON---------------------------------
//-------------------------------------------------------------------------------
void WaterTank_Worker::update_com()
{
    jetson_link.updateFrame();//this is a non-blocking function! 
    if (jetson_link.hasNewFrame())
    {
        Frame f;
        jetson_link.getFrame(f);
        process_request(f); 
        //process request:
    }
}

void WaterTank_Worker::process_request(const Frame& f)
{
    Frame reply;
    uint8_t dest = f.header.src;   // reply goes back to whoever asked

    switch (f.header.type)
    {
        case PING_:
            buildHeader(reply, PONG, 0, dest);
            jetson_link.sendFrame(reply);
            break;

        case DATA_ASK:
            if (f.payload.data.cmdId == ASK_TANK_STATE)
            {
                 
                buildHeader(reply, DATA_SEND, 2, dest);
                reply.payload.data.cmdId  = ASK_TANK_STATE;
                reply.payload.data.param[0] = (uint8_t)current_tank_state;//
                jetson_link.sendFrame(reply);
            }
            break;

        case CMD:
            switch (f.payload.data.cmdId)
            {
                case START_CYCLE:
                    startCycleFlag = true;   // business FSM picks this up
                    buildHeader(reply, ACK, 2, dest);
                    reply.payload.action_ack.cmdId  = START_CYCLE;
                    reply.payload.action_ack.status = ACK_OK;
                    jetson_link.sendFrame(reply);
                    break;

                case WAKE_UP:
                    buildHeader(reply, ACK, 2, dest);
                    reply.payload.action_ack.cmdId  = WAKE_UP;
                    reply.payload.action_ack.status = ACK_OK;
                    jetson_link.sendFrame(reply);
                    break;
            }
            break;
    }
}

void WaterTank_Worker::buildHeader(Frame& f, MsgType type, uint8_t len, uint8_t dest)
{
    f.sof         = SOF_VALUE;
    f.header.src  = TANK_MCU;
    f.header.dest = dest;
    f.header.type = type;
    f.header.len  = len;
}

//<<<--------------------DEBUG HELPERS----------------------->>>
void WaterTank_Worker::debug_heartbeat()
{
#if DEBUG_SERIAL
    static unsigned long last = 0;
    if (millis() - last < 1000) return;
    last = millis();

    Serial.print(F("   . "));
    Serial.print(state_name(current_tank_state));
    Serial.print(F("  t="));
    Serial.print(time_in_state() / 1000);
    Serial.print(F("s   up="));
    Serial.println(sensor_up_val);
    //Serial.print(F("  down="));
    //Serial.println(sensor_down_val);
#endif
}


void WaterTank_Worker::debug_console()
{
#if DEBUG_SERIAL
    if (!Serial.available()) return;
    char c = Serial.read();
    switch (c) {
        case 'p': pumpON();      Serial.println(F(">> PUMP ON"));    break;
        case 'P': pumpOFF();     Serial.println(F(">> PUMP OFF"));   break;
        case 'v': open_valve();  Serial.println(F(">> VALVE OPEN"));  break;
        case 'V': close_valve(); Serial.println(F(">> VALVE CLOSE")); break;
        case 'c': startCycleFlag = true;
                  Serial.println(F(">> START_CYCLE (simulated)"));    break;
        case 's': read_sensors();
                  Serial.print(F(">> up="));   Serial.print(sensor_up_val);
                  //Serial.print(F(" down="));   Serial.print(sensor_down_val);
                  Serial.print(F("  isFull=")); Serial.print(isFull());
                  Serial.print(F(" isEmpty=")); Serial.println(isEmpty());
                  break;
    }
#endif
}


const char* WaterTank_Worker::state_name(tank_state s)
{
    switch (s) {
        case tank_state::CHECK_STABILITY: return "CHECK_STABILITY";
        //case tank_state::DRAIN_AT_TOP:    return "DRAIN_AT_TOP";
        case tank_state::FILL:            return "FILL";
        case tank_state::READY:           return "READY";
        case tank_state::DRAIN:           return "DRAIN";
    }
    return "UNKNOWN";
}