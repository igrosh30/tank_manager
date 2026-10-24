#include "WaterTank_Worker.hpp"

WaterTank_Worker worker;

void setup()
{
    worker.init(&Serial);
    pinMode(LED_BUILTIN, OUTPUT);
    
    //check COM and Make tank stable! 
    
    auto entry = millis();
    bool setup_ready = false;
    do
    {
        if(!worker.com_working)
            worker.com_working=worker.setupCOM();
        if(!worker.tank_stable)
            worker.tank_stable=worker.setup_env();
    
        if(worker.com_working)
        {
            worker.update_com();//listen and respond to commands as soon as possible 
        }
        if (millis() % 200 < 100)
            digitalWrite(LED_BUILTIN, HIGH);
        else
            digitalWrite(LED_BUILTIN, LOW);
    setup_ready = (worker.com_working&& worker.tank_stable) || (millis()-entry >= SETUP_TIMEOUT_MS);
    
    /*look at the condition to leave this:
    1- setupCom done + setup_env() - ✅
    2- on Timeout ⏰ -> *check what failled! 
    */
    } while(!setup_ready);
    

}
void loop()
{
    worker.update_com();
    worker.update_state();
}

