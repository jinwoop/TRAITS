/* Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Modeling and Optimizing the Provisioning of Exhaustible Capabilities
 * for Simultaneous Task Allocation and Scheduling
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 * Author: Jinwoo Park
 *
 * Copyright (C) 2020–2023 Andrew Messing
 * Copyright (C) 2020–2023 Glen Neville
 * Copyright (C) 2026 Jinwoo Park
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
#include "traits/robot.hpp"

// Local
#include "traits/geometric_planning/configurations/configuration_base.hpp"
#include "traits/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "traits/species.hpp"

namespace traits
{
    unsigned int Robot::s_num_robots = 0;
    unsigned int Robot::s_next_id    = 0;

    Robot::Robot(const std::string& name,
                         const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                         const std::shared_ptr<const Species>& species,
                         const Eigen::VectorXf& initial_trait_levels,
                         float initial_battery_level,
                         unsigned int numTraits,
                         unsigned int numTasks)
        : m_id(s_next_id++)
        , m_name(name)
        , m_initial_configuration(initial_configuration)
        , m_species(species)
        , m_initial_battery_level(initial_battery_level * species->max_battery_capacity())
        , m_initial_traits(initial_trait_levels.asDiagonal() * species->traits_max())
        , m_inter_transition_velocity(-1.0f)
    {
        ++s_num_robots;
    }

    Robot::~Robot()
    {
        // If all robots have been destructed reset the next id
        --s_num_robots;
        if(s_num_robots == 0)
        {
            s_next_id = 0;
        }
    }

    float Robot::boundingRadius() const
    {
        return m_species->boundingRadius();
    }

    float Robot::max_speed() const
    {
        return m_species->max_speed();
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> Robot::motionPlanningQuery(
        const std::shared_ptr<const ConfigurationBase>& initial,
        const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return m_species->motionPlanner()->query(m_species, initial, terminal);
    }

    float Robot::pathLengthQuery(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return m_species->motionPlanner()->query(m_species, initial, terminal)->length();
    }

    float Robot::durationQuery(const std::shared_ptr<const ConfigurationBase>& initial,
                               const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return m_species->motionPlanner()->durationQuery(m_species, initial, terminal);
    }

    bool Robot::isMemoized(const std::shared_ptr<const ConfigurationBase>& initial,
                           const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return m_species->motionPlanner()->isMemoized(m_species, initial, terminal);
    }
}  // namespace traits