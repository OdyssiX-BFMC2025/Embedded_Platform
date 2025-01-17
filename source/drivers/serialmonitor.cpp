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


#include <drivers/serialmonitor.hpp>

namespace drivers{

    /** @brief  CSerialMonitor class constructor
     * @web This class implements a serial monitor for communication between devices.
     * It handles receiving, parsing and responding to serial messages in a specific format.
     *
     * @param f_serialPort Reference to the serial port object for communication
     * @param f_serialSubscriberMap Map containing message IDs and their callback handlers
     * 
     * Key member variables:
     * - m_RxBuffer: Buffer to store received data
     * - m_TxBuffer: Buffer to store data to be transmitted  
     * - m_parseBuffer: Temporary buffer for parsing messages
     * - m_parseIt: Iterator for parsing buffer
     * - m_serialSubscriberMap: Maps message IDs to handler functions
     */
    CSerialMonitor::CSerialMonitor(
            UnbufferedSerial& f_serialPort,
            CSerialSubscriberMap f_serialSubscriberMap)
        :utils::CTask(std::chrono::milliseconds(0))
        , m_serialPort(f_serialPort)
        , m_RxBuffer()
        , m_TxBuffer() 
        , m_parseBuffer()
        , m_parseIt(m_parseBuffer.begin())
        , m_serialSubscriberMap(f_serialSubscriberMap) 
        {
            // Attach RX interrupt handler
            m_serialPort.attach(mbed::callback(this,&CSerialMonitor::serialRxCallback), SerialBase::RxIrq);
            // TX interrupt handler commented out
            // m_serialPort.attach(mbed::callback(this,&CSerialMonitor::serialTxCallback), SerialBase::TxIrq);
        }

    /** @brief  CSerialMonitor class destructor
     * @web Empty destructor as no manual cleanup needed
     */
    CSerialMonitor::~CSerialMonitor()
    {
    };

    /** @brief  Rx callback actions
     * @web This interrupt handler is called when data arrives on serial port
     * - Disables other interrupts for thread safety
     * - Reads available bytes into RX buffer if space available
     * - Re-enables interrupts when done
     * For mutual exclusion, we use the __disable_irq() and __enable_irq() functions.
     */
    void CSerialMonitor::serialRxCallback()
    {
        __disable_irq();
        while ((m_serialPort.readable()) && (!m_RxBuffer.isFull())) {
            char buf;
            m_serialPort.read(&buf, 1);
            m_RxBuffer.push(buf);
        }
        __enable_irq();
        return;
    }

    /** @brief  Tx callback actions
     * @web This interrupt handler manages serial transmission
     * - Disables interrupts for thread safety
     * - Writes data from TX buffer when port is ready
     * - Currently only writes test char 'a'
     * - Re-enables interrupts when done
     */
    void CSerialMonitor::serialTxCallback()
    {
        __disable_irq();
        while ((m_serialPort.writeable()) && (!m_TxBuffer.isEmpty())) {
            // m_serialPort.write(m_TxBuffer.pop(), 1);
            m_serialPort.write("a", 1);
        }
        __enable_irq();
        return;
    }

    /** @brief  Main message processing loop
     * @web Processes received data character by character:
     * 
     * Variables used:
     * - l_c: Current character being processed
     * - l_msgID: Buffer to store message identifier (max 64 chars)
     * - l_msg: Buffer to store message content (max 64 chars)
     * - l_resp: Buffer for response message (max 128 chars)
     * - formattedResp: Final formatted response string (max 256 chars)
     * 
     * Message format: #ID:MESSAGE;;\r\n
     * 1. Waits for # to start new message
     * 2. Collects characters until \n found
     * 3. Validates message ends with ;;\r\n
     * 4. Parses into ID and message components
     * 5. Looks up handler for ID
     * 6. Calls handler with message
     * 7. Formats and sends response as @ID:RESPONSE;;\r\n
     */
    void CSerialMonitor::_run()
    {
        if ((!m_RxBuffer.isEmpty()))
        {
            char l_c = m_RxBuffer.pop(); // Get next character from buffer

            if ('#' == l_c) // Start new message when # found
            {
                m_parseIt = m_parseBuffer.begin();
                m_parseIt[0] = l_c;
                m_parseIt++;
                return;
            }
            if (m_parseIt != m_parseBuffer.end())
            {
                if (l_c == '\n') // End of message found
                {
                    // Check for proper message termination ;;\r\n
                    if ((';' == m_parseIt[-3]) && (';' == m_parseIt[-2]) && ('\r' == m_parseIt[-1])) 
                    {
                        char l_msgID[64];  // Buffer for message ID
                        char l_msg[64];    // Buffer for message content

                        // Parse message into ID and content
                        // sscanf parses the buffer into two parts:
                        // 1. %[^:] reads chars until ':' is found, stores in l_msgID
                        // 2. %[^\r\n] reads chars until \r or \n, stores in l_msg
                        // While int could be used here, uint32_t is preferred because:
                        // - sscanf returns non-negative values (items parsed or EOF)
                        // - uint32_t explicitly shows we only need positive values
                        // - Maintains consistency with other similar parsing code
                        // uint32_t range: 0 to 4,294,967,295 (2^32 - 1)
                        uint32_t res = sscanf(m_parseBuffer.data(), "#%[^:]:%[^\r\n]", l_msgID, l_msg);

                        if (res == 2) // If parsing successful
                        {
                            auto l_pair = m_serialSubscriberMap.find(l_msgID); // Find message handler
                            if (l_pair != m_serialSubscriberMap.end())
                            {
                                char l_resp[128] = {0}; // Response buffer

                                l_pair->second(l_msg,l_resp); // Call message handler
                                char formattedResp[256];
                                if (strlen(l_resp) > 0)
                                {
                                    // Format and send response
                                    snprintf(formattedResp, sizeof(formattedResp), "@%s:%s;;\r\n", l_msgID, l_resp);
                                    m_serialPort.write(formattedResp,strlen(formattedResp));
                                }
                            }
                        }
                        m_parseIt = m_parseBuffer.begin(); // Reset parser
                    }
                }
                m_parseIt[0] = l_c;  // Store current character
                m_parseIt++;
                return;
            }
        }
    }

}; // namespace drivers