#include "WaterTank_Worker.hpp"

WaterTank_Worker worker;

void setup()
{
    Serial.begin(9600);
    worker.init(&Serial);

    pinMode(LED_BUILTIN, OUTPUT);
}
void loop()
{
    #ifndef DEBUG_SERIAL
    worker.update_com();
    #endif

    worker.update_state();

    //Serial.print(worker.sensor_up_val);
    //worker.read_sensors();    
    //worker.debug_console();//T3 p/P/v/V/s commands
}

