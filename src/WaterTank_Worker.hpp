#pragma once

#include "Arduino.h"

//#define DEBUG_SERIAL 1

//#include "SerialTransport.h"
#include "SerialTransport.h"
#include "protocol.h"


//-----------GPIO DEF------------------//

/*
⚠️ SENSOR DOWN Is not working!
is empty using Sensor UP! 

liga bem desliga mal!!! 
*/
//constexpr uint8_t SENSOR_DOWN = 7;
constexpr uint8_t SENSOR_UP = 6;

constexpr uint8_t PUMP_FASE2 = 5;// and 5INI1 - VALVULA!
constexpr uint8_t PUMP_FASE1 = 4;// and 5INI1 - VALVULA!
constexpr uint8_t VALVE = 3;//INI2 

//THI IS NOT CONST! JETSON CAN DECIDE TO CHANGE THIS! - 

constexpr uint32_t MS_PER_MIN = 60UL * 1000UL;
constexpr uint32_t MS_PER_SEC= 1000UL;

constexpr uint32_t DRAIN_TIMEOUT_MS             = 1*MS_PER_MIN+ 30*MS_PER_SEC;//DRAIN_TIMEOUT
constexpr uint32_t DRAIN_LEFTOVERS_TIMEOUT_MS   = 30*MS_PER_SEC;//DRAIN_TIMEOUT
constexpr uint32_t FILL_TIMEOUT_MS              = 50*MS_PER_SEC;//it had already been pumping watter!   
constexpr uint32_t SENSOR_TIMEOUT_MS            = 30*MS_PER_MIN;  

//constexpr uint32_t SETUP_TIMEOUT_MS = DRAIN_TIMEOUT_MS+ FILL_TIMEOUT_MS + 2*MS_PER_MIN;; 


// ⚠️ MUST stay byte-identical with the Jetson's tank_state
enum class tank_state : uint8_t
{
    CHECK_STABILITY = 0,   // boot: read sensors, decide
    DRAIN_LEFTOVERS = 1,   // level too high — bleed down
    FILL            = 2,   // filling to level (shared by both phases)
    READY           = 3,   // levelled, idle, awaiting command
    DRAIN           = 4,   // full-cycle drain
};



class WaterTank_Worker
{    
    private:
       
    public:        
        
        bool gpio_active = false;// TODO: derive from .enabled after setup_gpios, or set true when GPIOs wired
        bool startCycleFlag = false;

        //I need to store the value of the actual motor right? ....
        uint16_t sensor_down_val;
        uint16_t sensor_up_val;
        
        //uint32_t time_sensorDownON;
        uint32_t time_sensorUpON; 

        SerialTransport jetson_link;
 

        //deploy_tank deploy_state;
        tank_state current_tank_state;
        tank_state previous_tank_state;
    
        unsigned long state_entry;

        //Setup funct:
        void init(Stream* port); // we do DigitalWrite, etc. 

        void read_sensors();
        void set_actuator(int gpio, int val);
        
        void update_state();
        void update_com();
        void process_request(const Frame& f);
        void buildHeader(Frame& f, MsgType type, uint8_t len, uint8_t dest);
        
        unsigned long time_in_state() const;
        void set_next_state(tank_state s, const char* reason);


        bool isEmpty() const;
        bool isTopEmpty() const;
        bool isFull()const;
        
        void pumpON();
        void pumpOFF();
        void open_valve();
        void close_valve();

        //----------DEBUG Helpers------------------//
        const char* state_name(tank_state s);
        void debug_heartbeat();
        void debug_console();
};
