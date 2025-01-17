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

namespace utils {

    /** \brief CTask class implements a periodic task execution system
     *
     * This class provides the base functionality for executing periodic tasks:
     * 
     * Flow:
     * 1. Task is created with a specified period in milliseconds
     * 2. timerCallback() is called every 1ms by the task manager
     * 3. When accumulated ticks reach the period, task is triggered
     * 4. run() checks if triggered and calls _run() implementation
     * 5. Cycle repeats
     *
     * Key member variables:
     * - m_period: Time between task executions in milliseconds
     * - m_ticks: Counter tracking elapsed time
     * - m_triggered: Flag indicating task should execute
     */
    CTask::CTask(std::chrono::milliseconds f_period)
        : m_period(f_period)          // Set task period
        , m_ticks(std::chrono::milliseconds(0))  // Initialize tick counter
        , m_triggered(false)          // Start with task not triggered
    {
    }

    /** \brief Empty destructor as no cleanup needed */
    CTask::~CTask()
    {
    }

    /** \brief Updates the task's execution period
     * 
     * @param f_period New period in milliseconds
     * Resets tick counter when period changes
     */
    void CTask::setNewPeriod(uint16_t f_period) 
    {
        m_period = std::chrono::milliseconds(f_period);
        m_ticks = std::chrono::milliseconds(0);
    }

    /** \brief Called every millisecond by task manager
     *
     * Increments tick counter and checks if period elapsed
     * When period reached:
     * 1. Resets tick counter
     * 2. Sets triggered flag for next run
     */
    void CTask::timerCallback()
    {
        m_ticks += std::chrono::milliseconds(1);
        if (m_ticks >= m_period)
        {
            m_ticks = std::chrono::milliseconds(0);
            m_triggered = true;
        }
    }

    /** \brief Main task execution method
     * 
     * Checks if task is triggered and needs to run
     * If triggered:
     * 1. Clears triggered flag
     * 2. Calls _run() virtual method implemented by derived class
     */
    void CTask::run()
    {
        if (m_triggered)
        {
            m_triggered = false;
            _run();
        }
    }

}; // namespace utils