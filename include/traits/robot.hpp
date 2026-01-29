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
#pragma once

// Global
#include <memory>
#include <string>
#include <eigen3/Eigen/Core>

namespace traits
{
    // Forward Declaration
    class Species;
    class ConfigurationBase;
    class MotionPlannerQueryResultBase;

    /*!
     * A container for the information about a single robot
     */
    class Robot
    {
       public:
        //! Constructor
        Robot(const std::string& name,
                  const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                  const std ::shared_ptr<const Species>& species,
                  const Eigen::VectorXf& initial_trait_levels,
                  float initial_battery_level,
                  unsigned int numTraits,
                  unsigned int numTasks);

        //! Destructor
        ~Robot();

        //! \returns The name of the robot
        [[nodiscard]] inline const std::string& name() const;

        //! \returns An identifier for the robot
        [[nodiscard]] inline unsigned int id() const;

        //! \returns The initial configuration of the robot
        [[nodiscard]] inline const std::shared_ptr<const ConfigurationBase>& initialConfiguration() const;

        //! \returns The species of the robot
        [[nodiscard]] inline const std::shared_ptr<const Species>& species() const;

        //! \returns The radius of a bounding circle/sphere for this robot
        [[nodiscard]] float boundingRadius() const;

        //! \returns The speed for this robot
        [[nodiscard]] float max_speed() const;

        //! \returns The initial battery level for this robot
        [[nodiscard]] inline double initial_battery_level() const;

        [[nodiscard]] inline float inter_transition_velocity() const;
        inline void set_inter_transition_velocity(float inter_transition_velocity) const;

        //! \returns The initial traits for this robot
        [[nodiscard]] inline const Eigen::VectorXf & initial_traits() const;

        /*!
         * \returns The result of a motion planning query for this robot from \p initial to \p terminal
         */
        [[nodiscard]] std::shared_ptr<const MotionPlannerQueryResultBase> motionPlanningQuery(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /*!
         * \returns The result of a motion planning query for this robot from its initial
         *          configuration to \p terminal
         */
        [[nodiscard]] inline std::shared_ptr<const MotionPlannerQueryResultBase> motionPlanningQuery(
            const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /*!
         * \returns The duration of a motion plan for this robot from \p initial to \p terminal
         */
        [[nodiscard]] float durationQuery(const std::shared_ptr<const ConfigurationBase>& initial,
                                          const std::shared_ptr<const ConfigurationBase>& terminal) const;


        [[nodiscard]] float pathLengthQuery(const std::shared_ptr<const ConfigurationBase>& initial,
                                          const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /*!
         * \returns The duration of a motion plan for this robot its initial
         *          configuration to \p terminal
         */
        [[nodiscard]] inline float durationQuery(const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /*!
         * \returns Whether the query for a motion plan from \p initial to \p terminal for a robot with the same
         *          bounding radius as this robot has been computed
         */
        [[nodiscard]] bool isMemoized(const std::shared_ptr<const ConfigurationBase>& initial,
                                      const std::shared_ptr<const ConfigurationBase>& terminal) const;

        /*!
         * \returns Whether the query for a motion plan from \p initial to \p terminal for a robot with the same
         *          bounding radius as this species has been computed
         */
        [[nodiscard]] inline bool isMemoized(const std::shared_ptr<const ConfigurationBase>& terminal) const;

       private:
        std::shared_ptr<const ConfigurationBase> m_initial_configuration;
        std::string m_name;
        std::shared_ptr<const Species> m_species;

        const Eigen::VectorXf m_initial_traits;

        unsigned int m_id;
        double m_initial_battery_level;
        mutable float m_inter_transition_velocity; // mutable needed for task duration calculation in MILP Scheduler

        static unsigned int s_num_robots;
        static unsigned int s_next_id;
    };

    // Inline functions
    const std::string& Robot::name() const
    {
        return m_name;
    }

    unsigned int Robot::id() const
    {
        return m_id;
    }

    double Robot::initial_battery_level() const
    {
        return m_initial_battery_level;
    }

    float Robot::inter_transition_velocity() const
    {
        return m_inter_transition_velocity;
    }

    void Robot::set_inter_transition_velocity(float inter_transition_velocity) const
    {
        m_inter_transition_velocity = inter_transition_velocity;
    }

    const Eigen::VectorXf & Robot::initial_traits() const
    {
        return m_initial_traits;
    }

    const std::shared_ptr<const ConfigurationBase>& Robot::initialConfiguration() const
    {
        return m_initial_configuration;
    }

    const std::shared_ptr<const Species>& Robot::species() const
    {
        return m_species;
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> Robot::motionPlanningQuery(
        const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return motionPlanningQuery(m_initial_configuration, terminal);
    }

    float Robot::durationQuery(const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return durationQuery(m_initial_configuration, terminal);
    }

    bool Robot::isMemoized(const std::shared_ptr<const ConfigurationBase>& terminal) const
    {
        return isMemoized(m_initial_configuration, terminal);
    }
}  // namespace traits