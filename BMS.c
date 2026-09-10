#include "HAL.h"

//fault thresholds
#define OVER_VOLTAGE_THRESH 4.2f
#define UNDER_VOLTAGE_THRESH 2.5f
#define OVER_TEMPERATURE_THRESH 60.0f
#define DELTA_EXCEEDED_THRESH 0.2f
#define OVER_CURRENT_THRESH  200000

//fault bits
#define CELL_OVER_VOLTAGE      (1U << 0)  //00000001
#define CELL_UNDER_VOLTAGE     (1U << 1)  //00000010
#define CELL_OVER_TEMPERATURE  (1U << 2)  //00000100
#define CELL_DELTA_EXCEEDED    (1U << 3)  //00001000
#define PACK_OVER_CURRENT      (1U << 4)  //00010000

//store current and remembered fualts
static uint8_t active_faults = 0;
static uint8_t latched_faults = 0;

static volatile int32_t pack_current_mA = 0;
static volatile bool current_received = false;

//only true when FAULTS_CLEAR message arrives
static volatile bool clear_requested = false;

void Init()
{
   HAL_SetSDC(false);


}


void Iter()
{
   // This function runs periodically at ~20Hz so 20 times per second
   float voltages[N_CELLS];
   float temp[N_CELLS];
   HAL_ReadVoltages(voltages);
   HAL_ReadTemperatures(temp);
   active_faults = 0;
   float min_voltage = voltages[0];
   float max_voltage = voltages[0];
   for(int i = 0; i < N_CELLS; i++){
        if(voltages[i] > OVER_VOLTAGE_THRESH) {
            active_faults |= CELL_OVER_VOLTAGE;
        }
        if(voltages[i] < UNDER_VOLTAGE_THRESH){
            active_faults |= CELL_UNDER_VOLTAGE;
        }
        if(temp[i] > OVER_TEMPERATURE_THRESH){
            active_faults |= CELL_OVER_TEMPERATURE;
        }
        if(voltages[i] > max_voltage){
            max_voltage = voltages[i];
        }
        if(voltages[i] <min_voltage){
            min_voltage = voltages[i];
        }
   }
   if((max_voltage - min_voltage) > DELTA_EXCEEDED_THRESH){
        active_faults |= CELL_DELTA_EXCEEDED;
   }
   if (current_received && pack_current_mA > OVER_CURRENT_THRESH)
    {
        active_faults |= PACK_OVER_CURRENT;
    }
    // Remember every fault that is currently active
    latched_faults |= active_faults;
    if(clear_requested){
        // Clear old faults, but keep faults that are still active
        latched_faults = active_faults;
        clear_requested = false;
    }





}


void RxCan()
{
   // Called every time a CAN frame is received on the bus, using an interrupt.
   // Keep in mind, this can be called at any point in the execution of your program.
   // You may not use any HAL_* functions here except HAL_RecvCanMsg,
   // which is how you can pull the message from the bus.
   // An example for pulling a CAN frame is shown below.
   uint8_t data[CAN_LEN];
   uint16_t id;
   HAL_RecvCanMsg(&id, data);
   // Check if this CAN message came from the current sensor.
   if(id == 0x511){

     //We combine the 4 bytes back into one 32-bit number.
    uint32_t raw_current =
            ((uint32_t)data[2] << 24) |
            ((uint32_t)data[3] << 16) |
            ((uint32_t)data[4] << 8)  |
            ((uint32_t)data[5]);  //combinig data 2 to 5 into one big number

         // The current sensor sends a SIGNED number.
        //
        // For a signed 32-bit number:
        // leftmost bit = 0 -> positive
        // leftmost bit = 1 -> negative
        //
        // 0x80000000U is:
        //
        // 10000000 00000000 00000000 00000000
        //
        // So "&" checks only the leftmost/sign bit.
        if (raw_current & 0x80000000U) //if true sign bit is 1 and current is negative
                                       // Subtracting 2^32 converts that unsigned value
                                       // into the correct negative two's-complement value.
                                       //
                                       // 2^32 = 4,294,967,296

        {
            pack_current_mA = (int32_t)((int64_t)raw_current - 4294967296LL);
        }
        else //sign bit was 0 so current is positive
        {
            pack_current_mA = (int32_t)raw_current;
        }

        current_received = true;  //recieved at least 1 real current measurement
    }
    else if(id == 0x1CF){
         // The diagnostic tool sent FAULTS_CLEAR.
         clear_requested = true;
    }
   }


