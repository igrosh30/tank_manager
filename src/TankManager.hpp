#include "Arduino.h"


constexpr uint8_t SENSOR_UP = 15;
constexpr uint8_t SENSOR_DOWN = 16;
constexpr uint8_t PUMP = 4;
constexpr uint8_t VALVE = 5;

//THI IS NOT CONST! JETSON CAN DECIDE TO CHANGE THIS! - 
constexpr uint32_t MS_PER_MIN = 60UL * 1000UL;
constexpr uint32_t DRAIN_TIMEOUT_MS  = 10 * MS_PER_MIN;
constexpr uint32_t FILL_TIMEOUT_MS   =  3 * MS_PER_MIN;
constexpr uint32_t SENSOR_TIMEOUT_MS =  1 * MS_PER_MIN; 


// enum class deploy_tank
// {
//     IDLE,
//     HALF_DRAIN,
//     FULL_DRAIN,
//     FILL,
//     STABLE,
// };

enum class tank_state
{
    IDLE,
    WAIT_CMD,
    DRAIN_AT_TOP,
    DRAIN,
    FILL,
    REPORT,
};

class WaterTank
{
    private:
    
    
    public:
        
        
        bool gpio_active = false;// TODO: derive from .enabled after setup_gpios, or set true when GPIOs wired

        //I need to store the value of the actual motor right? ....
        uint16_t sensor_down_val;
        uint16_t sensor_up_val;
        
        //Serial* jetson_com;

        //deploy_tank deploy_state;
        tank_state current_tank_state;
        unsigned long state_entry;

        //Setup funct:
        void init(); // we do DigitalWrite, etc. 
        bool setup_env();//this can also be runned on the setup()

        void read_sensors();
        void set_actuator(int gpio, int val);
        

        void update_state();
        
        unsigned long time_in_state() const;
        void set_next_state(tank_state s );


        bool startCycle();
        bool isEmpty() const;
        bool isTopEmpty() const;
        bool isFull()const;
        
        void pumpON();
        void pumpOFF();
        void open_valve();
        void close_valve();
};