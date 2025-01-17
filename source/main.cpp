/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/
/* Header file for the motion controller functionality */
#include <main.hpp>

/*
Overall Code Flow:
1. System Initialization
   - Load headers and define constants
   - Setup communication interfaces
   - Initialize all system components
2. Component Creation
   - Create monitoring systems (battery, IMU, etc)
   - Setup motion control systems
   - Initialize management systems
3. Command System Setup
   - Map commands to handlers
   - Setup serial communication
4. Task Management
   - Create task list
   - Initialize scheduler
5. Runtime Loop
   - Execute setup
   - Run periodic tasks
   - Monitor for shutdown/errors

Detailed Line-by-Line Explanation:
*/

// Define dummy value used for battery manager initialization
#define dummy_value 15

// Base tick rate (1 millisecond) used as the fundamental timing unit for all periodic tasks
// This determines the granularity of task scheduling
const std::chrono::milliseconds g_baseTick = std::chrono::milliseconds(1);

// Initialize serial communication with Raspberry Pi
// USBTX/USBRX: Hardware UART pins
// 115200: Baud rate for serial communication
UnbufferedSerial g_rpi(USBTX, USBRX, 115200);

// Create LED blinker task that toggles every 500ms (500 * base tick)
// LED1 is the onboard LED used to indicate system status
periodics::CBlinker g_blinker(g_baseTick * 500, LED1);

// Initialize alert system with 5 second check period
periodics::CAlerts g_alerts(g_baseTick * 5000);

// Setup battery current consumption monitor
// Updates every 1 second, reads from analog pin A2
periodics::CInstantConsumption g_instantconsumption(g_baseTick * 1000, A2, g_rpi);

// Setup battery voltage monitor
// Updates every 3 seconds, reads from analog pin A4
periodics::CTotalVoltage g_totalvoltage(g_baseTick*3000, A4, g_rpi);

// Initialize IMU sensor communication
// Updates every 150ms, uses I2C interface
periodics::CImu g_imu(g_baseTick*150, g_rpi, I2C_SDA, I2C_SCL);

// Configure motor speed controller
// D3: PWM pin for speed control
// Range: -500 to 500 cm/s
drivers::CSpeedingMotor g_speedingDriver(D3, -500, 500);

// Configure steering servo controller
// D4: PWM pin for steering
// Range: -250 to 250 degrees
drivers::CSteeringMotor g_steeringDriver(D4, -250, 250);

// Initialize robot state machine for motion control
// Updates every 50ms
// Create robot state machine object:
// - brain::CRobotStateMachine: Class type from brain namespace
// - g_robotstatemachine: Global variable name
// - Constructor parameters:
//   1. g_baseTick * 50: Update period of 50ms (base tick = 1ms)
//   2. g_rpi: Serial communication interface with Raspberry Pi
//   3. g_steeringDriver: Motor controller for steering
//   4. g_speedingDriver: Motor controller for speed/acceleration
brain::CRobotStateMachine g_robotstatemachine(g_baseTick * 50, g_rpi, g_steeringDriver, g_speedingDriver);

// Setup resource monitoring system
// Checks system resources every 5 seconds
periodics::CResourcemonitor g_resourceMonitor(g_baseTick * 5000, g_rpi);

// Initialize keyless manager to handle security features
brain::CKlmanager g_klmanager(g_alerts, g_imu, g_instantconsumption, g_totalvoltage, g_robotstatemachine, g_resourceMonitor);

// Setup power management system
// Monitors power status every 100ms
periodics::CPowermanager g_powermanager(g_baseTick * 100, g_klmanager, g_rpi, g_totalvoltage, g_instantconsumption, g_alerts);

// Initialize battery management system
brain::CBatterymanager g_batteryManager(dummy_value);

/* USER NEW COMPONENT BEGIN */
/* This section is reserved for adding new component declarations and initializations.
   Components are typically hardware interfaces, sensors, or system modules that need
   to be instantiated before use. They would be declared as global variables here,
   similar to other components like g_alerts, g_imu, etc. above. */

/* USER NEW COMPONENT END */

// Map commands to their respective handler functions
// Each entry pairs a command string with its callback function
drivers::CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    {"speed",          mbed::callback(&g_robotstatemachine, &brain::CRobotStateMachine::serialCallbackSPEEDcommand)},
    {"steer",          mbed::callback(&g_robotstatemachine, &brain::CRobotStateMachine::serialCallbackSTEERcommand)},
    {"brake",          mbed::callback(&g_robotstatemachine, &brain::CRobotStateMachine::serialCallbackBRAKEcommand)},
    {"vcd",            mbed::callback(&g_robotstatemachine, &brain::CRobotStateMachine::serialCallbackVCDcommand)},
    {"battery",        mbed::callback(&g_totalvoltage,      &periodics::CTotalVoltage::serialCallbackTOTALVcommand)},
    {"instant",        mbed::callback(&g_instantconsumption,&periodics::CInstantConsumption::serialCallbackINSTANTcommand)},
    {"imu",            mbed::callback(&g_imu,               &periodics::CImu::serialCallbackIMUcommand)},
    {"kl",             mbed::callback(&g_klmanager,         &brain::CKlmanager::serialCallbackKLCommand)},
    {"batteryCapacity",mbed::callback(&g_batteryManager,    &brain::CBatterymanager::serialCallbackBATTERYCommand)},
    {"resourceMonitor",mbed::callback(&g_resourceMonitor,   &periodics::CResourcemonitor::serialCallbackRESMONCommand),}
};

// Initialize serial monitor to handle command processing
drivers::CSerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

// Create array of tasks to be executed periodically
utils::CTask* g_taskList[] = {
    &g_blinker,
    &g_instantconsumption,
    &g_totalvoltage,
    &g_imu,
    &g_robotstatemachine,
    &g_serialMonitor,
    &g_powermanager,
    &g_resourceMonitor,
    &g_alerts,
    // USER NEW PERIODICS BEGIN -
    
    // USER NEW PERIODICS END
}; 

// Initialize task manager with task list and base tick rate
utils::CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(utils::CTask*), g_baseTick);

// Setup function - runs once at startup
uint8_t setup()
{
    // Send startup message over serial
    g_rpi.write("\r\n\r\n", 4);
    g_rpi.write("#################\r\n", 19);
    g_rpi.write("#               #\r\n", 19);
    g_rpi.write("#   I'm alive   #\r\n", 19);
    g_rpi.write("#               #\r\n", 19);
    g_rpi.write("#################\r\n", 19);
    g_rpi.write("\r\n", 2);

    return 0;    
}

// Main loop function - called repeatedly
uint8_t loop()
{
    g_taskManager.mainCallback();
    return 0;
}

// Main program entry point
int main() 
{   
    uint8_t l_errorLevel = setup();

    while(!l_errorLevel) 
    {
        if(bool_globalsV_ShuttedDown) {
            ThisThread::sleep_for(chrono::milliseconds(200));
            hal_deepsleep();
        }
        l_errorLevel = loop();
    }
    
    return l_errorLevel;
}