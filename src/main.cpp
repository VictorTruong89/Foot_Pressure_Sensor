// Example code how to read the serial port and process the pressure data from the foot sensor


#include <iostream>
#include <string.h>
#include <chrono>
// Command Parser
#include <unistd.h>
#include <getopt.h>

#include "foot_sensor.hpp"


bool use_foot_sensor = true;    // Flag from the command parser

int main(int argc, char** argv)
{
    // To calculate pressure data
    FootSensor foot_sensor;

    // struct to store data
    PressureData pressure_data;

    // Initialize serial communication
    USBStream* serial_port;
    serial_port = new USBStream[2];

    if(use_foot_sensor)
    {
        // Initialize serial communications
        foot_sensor.OpenSerialPort(serial_port);    // Serial-port name is defined inside foot_sensor.cpp

        // Initialize FileterSpike in pressure reading
        foot_sensor.FilterSpike_Init(serial_port, &pressure_data);
    }
    
    bool spike_check[2] = {true, true};

    // Initialize Heel Strike checking
    int heel_strike = 0;    // 0 -> nothing ; 1 -> left-heel-strike ; 2 -> right
    int heel_check[2] = {0, 0}; // Denote 2 consecutive times checking pressure-gradiant surge

    /*===================== INITIALIZE WHILE LOOP =====================*/
    auto program_start = std::chrono::steady_clock::now();
    int num_loop = 0;
    // Flag of step_complete
    bool step_complete = false;
    bool right_leading;

    // Return the time interval between 2 CAN BUS reading (in seconds)
    float time_interval;

    while(true)
    {
        if(use_foot_sensor)
        {
            // Read foot-sensor 99 pixels & store into PressureData struct
            foot_sensor.ReadPressureData(serial_port, &pressure_data);
            std::cout << "[SWING PHASE]\tRight Pressure Sum = " << pressure_data.right_pressure;

            // Calculate the COP of 2 foot-sensors
            foot_sensor.CalcCOP(&pressure_data);

            // Check heel strike
            foot_sensor.getHeelStrike(&pressure_data, &heel_check[0]);
        }
        std::cout << "\r";

        // Get another key-stroke for either step_complete or forced_stop
        char kb_press = kb.getNonBlockingTriggers();

        // Save realtime data
        exoskeleton.SaveFile(&save_file, &exo_data, &exo_cmd, &pressure_data, program_start);

        // Check for step-complete to end the SWING-PHASE while loop
        step_complete = exoskeleton.checkStepComplete(traj_type, &exo_cmd, heel_strike, kb_press);

        // Forced stop the program
        if(kb_press == KB_ESCAPE)
            break;
    }

    save_file.end();

    return 0;
}