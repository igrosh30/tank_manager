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
    if(port == nullptr) return;
    jetson_link.init(port,false);

    //----------PIN INITIALISATION-----------//
    pinMode(SENSOR_UP, INPUT);
    pinMode(SENSOR_DOWN,INPUT);
    pinMode(PUMP,OUTPUT);
    pinMode(VALVE,OUTPUT);

    //stability:
    this->pumpOFF();
    this->close_valve();
    set_next_state(tank_state::CHECK_STABILITY);//NEEDS to receive a PING! 

}
bool WaterTank_Worker::setupCOM()
{
    
    jetson_link.updateFrame();//this is a non-blocking function! 
    if (jetson_link.hasNewFrame())
    {
        jetson_link.getFrame(incomingFrame);
        if(incomingFrame.header.type== MsgType::PING)
        {
            Frame f;
            buildHeader(f,MsgType::PONG,0,incomingFrame.header.src);
            jetson_link.sendFrame(f);
            com_working = true;
        }
        //process request:
    }   
    return com_working;
}

//-------------------------------------------------------------------------------
//-------------------------GPIO FUNCTIONS----------------------------------------
//-------------------------------------------------------------------------------
void WaterTank_Worker::read_sensors()
{
    sensor_down_val=digitalRead(SENSOR_DOWN);
    sensor_up_val = digitalRead(SENSOR_UP);
}

bool WaterTank_Worker::isEmpty() const {return sensor_down_val == LOW;}
bool WaterTank_Worker::isTopEmpty() const {return sensor_up_val == LOW;}
bool WaterTank_Worker::isFull() const {return sensor_up_val == HIGH;}

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
    this->set_actuator(PUMP,HIGH);//check if 0 is the val to turn off the valve! 
}
void WaterTank_Worker::pumpOFF()
{
    this->set_actuator(PUMP,LOW);//check if 0 is the val to turn off the valve! 
}


//-------------------------------------------------------------------------------
//------------------------F.State Machines---------------------------------------
//-------------------------------------------------------------------------------


bool WaterTank_Worker::setup_env()
{
    switch (this->current_tank_state)
    {   
        case tank_state::CHECK_STABILITY:
            if(isFull())
            {
                this->open_valve();
                set_next_state(tank_state::DRAIN_AT_TOP);
            }
            else if(isEmpty()) // It's EMPTY! - FILL
            {
                this->pumpON();
                set_next_state(tank_state::FILL);
            }
            else//it's in the midle- we don't care! 
            {
                set_next_state(tank_state::WAIT_CMD);
            }
            break;

        case tank_state::DRAIN_AT_TOP:
            if(isTopEmpty())
            {
                this->close_valve();
                set_next_state(tank_state::WAIT_CMD);
            }
            else if(time_in_state() >= DRAIN_TIMEOUT_MS)//just a funct- timeout(timeout)
            {
                set_next_state(tank_state::FILL);
                this->close_valve();
                this->pumpON();
            }
            break;

        case tank_state::FILL:
            if(time_in_state() >= FILL_TIMEOUT_MS || isFull())
            {
                this->pumpOFF();
                set_next_state(tank_state::WAIT_CMD);
            }
            break;
    } 

   return this->current_tank_state == tank_state::WAIT_CMD;  
}
void WaterTank_Worker::update_state()//needs access to sensors! 
{       
    switch(this->current_tank_state)
    {
        case tank_state::WAIT_CMD: //waiting message to start! -> we can or not have a timeout also!
            //need to put listening for a valid CMD!! 
            if(this->startCycleFlag || this->time_in_state() >= SENSOR_TIMEOUT_MS)
            {
                this->open_valve();
                set_next_state(tank_state::DRAIN);
            }
            break;
        
        case tank_state::DRAIN:
            
            if( (time_in_state() >= DRAIN_TIMEOUT_MS) || isEmpty())
            {
                this->close_valve();
                this->pumpON();
                set_next_state(tank_state::FILL);
            }
            break;
        
        case tank_state::FILL: 
            if( (time_in_state()>= FILL_TIMEOUT_MS) || isFull())
            {
                this->pumpOFF();
                set_next_state(tank_state::STABLE);
            }
            break;
        case tank_state::STABLE:
            //we do a report and wait response - get the response - WAIT_CMD! 
            //if ack-> set_next_state(WAIT_CMD);
            break;
    }
}

void WaterTank_Worker:: set_next_state(tank_state s )
{
    if(s != current_tank_state)
    {
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
        jetson_link.getFrame(incomingFrame);
        process_request(incomingFrame);
        //process request:
    }

}

void WaterTank_Worker::process_request(const Frame& f)
{
    Frame reply;
    uint8_t dest = f.header.src;   // reply goes back to whoever asked

    switch (f.header.type)
    {
        case PING:
            buildHeader(reply, PONG, 0, dest);
            jetson_link.sendFrame(reply);
            break;

        case DATA_ASK:
            if (f.payload.data.cmdId == ASK_TANK_STATE)
            {
                buildHeader(reply, DATA_SEND, 2, dest);
                reply.payload.data.cmdId  = ASK_TANK_STATE;
                reply.payload.data.param[0] = (uint8_t)current_tank_state;
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
