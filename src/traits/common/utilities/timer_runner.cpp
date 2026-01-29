/* Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020–2023
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "traits/common/utilities/timer_runner.hpp"

// Local
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/time_keeper.hpp"

namespace traits
{
    TimerRunner::TimerRunner(const std::string& name)
            : m_name(name)
    {
        TimeKeeper::instance().setActive(name, &m_timer);
        m_timer.start();
    }

    TimerRunner::~TimerRunner()
    {
        m_timer.stop();
        TimeKeeper::instance().increment(m_name, m_timer.get());
        TimeKeeper::instance().setInactive(m_name, &m_timer);
    }
}  // namespace traits
