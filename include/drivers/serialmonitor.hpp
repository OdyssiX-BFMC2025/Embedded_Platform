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

/* Inclusion guard */
#ifndef SERIAL_MONITOR_HPP
#define SERIAL_MONITOR_HPP

/* The mbed library */
#include <mbed.h>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/taskmanager.hpp>
/* Header file for the queue manager library*/
#include <utils/queue.hpp>

#include <map>
#include <array>
#include <string>
#include <functional>
#include <chrono>


namespace drivers
{
   /**
    * @brief Class Serial Monitor
    * 
    * It aims to decode the messages received from the other device and redirectionate to other functions the content of message. 
    * For decode it has a predefined structure with a header (key) part and a content part. The key has to be four character, the content is defined by user.
    * The message received has to start with the '#' special character, the responses have the same key and start with "@" character. The special characters 
    * notice the direction of the message. Examples of messages:
    * 
    *   "#KEY1:MESSAGECONTENT;;\r\n"
    * 
    *   "@KEY1:RESPONSECONTANT;;\r\n"
    * 
    * The key differs for each functionalities, so for each callback function.
    */
    /**
     * @brief Serial Monitor class for handling serial communication
     * 
     * This class inherits from CTask and implements a serial monitor that:
     * 1. Receives messages in format: #ID:MESSAGE;;\r\n
     * 2. Parses and validates the message format
     * 3. Calls appropriate handler function based on message ID
     * 4. Sends response in format: @ID:RESPONSE;;\r\n
     *
     * Flow:
     * - Interrupt handler serialRxCallback() stores incoming bytes in RxBuffer
     * - Main _run() loop processes RxBuffer contents:
     *   a. Looks for # to start new message
     *   b. Collects chars until \n found
     *   c. Validates message format
     *   d. Parses ID and message content
     *   e. Calls registered handler function
     *   f. Formats and sends response
     * - serialTxCallback() handles sending data from TxBuffer
     */
    class CSerialMonitor : public utils::CTask
    {
        public:
            // Callback function type that takes const char* input and char* output
            typedef mbed::Callback<void(char const *, char *)> FCallback;
            
            // Map type to store message ID -> handler function mappings
            typedef std::map<string,FCallback> CSerialSubscriberMap;

            /* Constructor takes serial port and map of message handlers */
            CSerialMonitor(
                UnbufferedSerial& f_serialPort,
                CSerialSubscriberMap f_serialSubscriberMap
            );
            
            /* Virtual destructor */
            ~CSerialMonitor();

        private:
            /* Interrupt handler for receiving serial data */
            void serialRxCallback();
            
            /* Interrupt handler for transmitting serial data */
            void serialTxCallback();
            
            /* Main processing loop inherited from CTask */
            virtual void _run();

            /** @brief Reference to serial port for communication */
            UnbufferedSerial& m_serialPort;
            
            /** @brief Circular buffer for received data */
            utils::CQueue<char,255> m_RxBuffer;
            
            /** @brief Circular buffer for data to transmit */
            utils::CQueue<char,255> m_TxBuffer;
            
            /** @brief Buffer for message parsing */
            array<char,256> m_parseBuffer;
            
            /** @brief Iterator for parsing buffer position */
            array<char,256>::iterator m_parseIt;
            
            /** @brief Maps message IDs to their handler functions */
            CSerialSubscriberMap m_serialSubscriberMap;
    }; // class CSerialMonitor

}; // namespace drivers

#endif // SERIAL_MONITOR_HPP
