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


#include <utils/taskmanager.hpp>

namespace utils{
    /** \brief CTaskManager class implements a task scheduling and execution system
     *
     * The CTaskManager class manages multiple periodic tasks and handles their execution:
     *
     * Key components:
     * - Task List: Array of CTask pointers to manage multiple tasks
     * - Base Timer: Ticker that triggers task updates at specified frequency
     * - Task Count: Number of tasks being managed
     *
     * Flow:
     * 1. Constructor initializes task list and starts base timer
     * 2. Timer triggers timerCallback() at specified frequency (e.g. every 1ms)
     * 3. timerCallback() updates all tasks' internal timers
     * 4. mainCallback() executes tasks that are ready to run
     * 5. Process repeats
     */
    CTaskManager::CTaskManager(
            utils::CTask** f_taskList, 
            uint8_t f_taskCount, 
            std::chrono::milliseconds f_baseFreq
        )
        : m_taskList(f_taskList)      // Store task list pointer
        , m_taskCount(f_taskCount)    // Store number of tasks
    {
        // Start timer to call timerCallback at specified frequency
        m_ticker.attach(mbed::callback(this,&CTaskManager::timerCallback), f_baseFreq);
    }

    /** \brief Destructor stops the timer when object is destroyed */
    CTaskManager::~CTaskManager() 
    {
        m_ticker.detach(); // Stop the timer
    }
    
    /** \brief Executes all tasks that are ready to run
     *
     * Called periodically by main loop to:
     * 1. Iterate through all tasks
     * 2. Call run() on each task
     * 3. Task only executes if its period has elapsed
     */
    void CTaskManager::mainCallback()
    {
        for(uint8_t i = 0; i < m_taskCount; i++)
        {
            m_taskList[i]->run();
        }
    }

    /** \brief Updates all tasks' internal timers
     * 
     * Called by base timer to:
     * 1. Iterate through all tasks
     * 2. Update each task's tick counter
     * 3. Set triggered flag if task period elapsed
     */
    void CTaskManager::timerCallback()
    {
        for(uint8_t i = 0; i < m_taskCount; i++)
        {
            m_taskList[i]->timerCallback();
        }
    }

}; // namespace utils::task