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
#include <map>
#include <memory>
#include <mutex>
// External
#include <nlohmann/json.hpp>
#include <ompl/geometric/SimpleSetup.h>
// Local
#include "traits/geometric_planning/query_results/ompl_motion_planner_query_result.hpp"
#include "traits/common/utilities/noncopyable.hpp"
#include "traits/species.hpp"

// External Forward Declarations
namespace ompl
{
    namespace base
    {
        class SpaceInformation;
        class PlannerStatus;
        class Planner;
        class GoalState;
        class GoalStates;
        class GoalSpace;
    }  // namespace base

    namespace geometric
    {
        class PathGeometric;
    }  // namespace geometric
}  // namespace ompl

namespace traits
{
    // Forward Declarations
    class ConfigurationBase;
    class TraitsEnvironmentBase;
    class TraitsOmplEnvironment;
    class ParametersBase;
    class MotionPlannerQueryResultBase;
    enum class OmplMotionPlannerType : uint8_t;
    enum class ConfigurationType : uint8_t;

    /*!
     *  \brief Conducts motion planning by wrapping several classes from the Open Motion Planning Library
     *
     *  \cite I. Șucan, M. Moll, and L. Kavraki, "The Open Motion Planning Library",
     *        IEEE Robotics & Automation Magazine, 19(4):72–82, December 2012. https://ompl.kavrakilab.org
     */
    class TraitsOmplMotionPlanner : public Noncopyable
    {
    public:
        //! \Constructor
        TraitsOmplMotionPlanner(OmplMotionPlannerType ompl_motion_planner_type,
                          const std::shared_ptr<const ParametersBase>& parameters,
                          const std::shared_ptr<TraitsOmplEnvironment>& environment);

        /*!
         * \brief Queries for a path from \p initial_configuration to \p goal_configuration
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The result of the motion planning query
         */
        [[nodiscard]] std::shared_ptr<const MotionPlannerQueryResultBase> query(
                const std::shared_ptr<const Species>& species,
                const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                const std::shared_ptr<const ConfigurationBase>& goal_configuration);

        /*!
         * \brief Queries for a valid state
         *
         * \param species The species of the robot
         * \param target_configuration The geometric configuration of the robot
         *
         * \returns The result of the state being valid for inquired species
         */

        /*!
         * \brief Queries for the duration to execute the path from \p initial_configuration to \p
         *        goal_configuration
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The status of the planner and the path generated as the solution if possible
         */
        [[nodiscard]] float durationQuery(const std::shared_ptr<const Species>& species,
                                          const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                          const std::shared_ptr<const ConfigurationBase>& goal_configuration);

        /*!
         * \brief Checks if a path from \p start_state to \p goal_configuration has been memoized
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns Whether a path from \p start_state to \p goal_state has been memoized
         */
        [[nodiscard]] virtual bool isMemoized(const std::shared_ptr<const Species>& species,
                                              const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                              const std::shared_ptr<const ConfigurationBase>& goal_configuration) const;

        //! Initialize Factory
        static void init();

        //! \returns A pointer to the space information
        [[nodiscard]] const std::shared_ptr<ompl::base::SpaceInformation>& spaceInformation() const;

        //! \returns The type of motion planning algorithm used
        [[nodiscard]] inline OmplMotionPlannerType omplMotionPlannerType() const;

        //! \returns The parameters for this motion planner
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& parameters() const;

        //! \returns A pointer to the environment representation
        [[nodiscard]] inline const std::shared_ptr<TraitsEnvironmentBase>& environment() const;

        //! Clears the cache of previously computed motion plans
        void clearCache();

        //! \returns The number of motion plans computed
        [[nodiscard]] inline unsigned int numMotionPlans() const;

        //! \returns The number of times motion planning failed to find a solution
        [[nodiscard]] static unsigned int numFailures();

    protected:
        /*!
         * \brief Computes a motion plan
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The computed motion planning result
         */
        //! Computes a motion plan using an OMPL motion planner
        [[nodiscard]] std::shared_ptr<const MotionPlannerQueryResultBase> computeMotionPlan(
                const std::shared_ptr<const Species>& species,
                const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                const std::shared_ptr<const ConfigurationBase>& goal_configuration);

        /*!
         * \brief Returns the previously computed result of a motion planning query if one exists
         *
         * \param species The species of the robot
         * \param initial_configuration The initial geometric configuration of the robot
         * \param goal_configuration The target geometric configuration of the robot
         *
         * \returns The previously computed result of a motion planning query if one exists, nullptr otherwise
         */
        [[nodiscard]] std::shared_ptr<const MotionPlannerQueryResultBase> getMemoized(
                const std::shared_ptr<const Species>& species,
                const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                const std::shared_ptr<const ConfigurationBase>& goal_configuration) const;


        using MemoizationValue = std::tuple<std::shared_ptr<const ConfigurationBase>,
                std::shared_ptr<const ConfigurationBase>,
                std::shared_ptr<const MotionPlannerQueryResultBase>>;

        OmplMotionPlannerType m_ompl_motion_planner_type;
        std::unique_ptr<ompl::geometric::SimpleSetup> m_simple_setup;
        std::shared_ptr<const ParametersBase> m_parameters;
        std::shared_ptr<TraitsEnvironmentBase> m_environment;
        std::multimap<std::weak_ptr<const Species>, MemoizationValue, std::owner_less<>> m_memoization;
        mutable std::mutex m_mutex;  //!< mutable so that it can be used to lock const functions

        static unsigned int s_num_failures;
    };

    // Inline Functions
    OmplMotionPlannerType TraitsOmplMotionPlanner::omplMotionPlannerType() const
    {
        return m_ompl_motion_planner_type;
    }

    const std::shared_ptr<const ParametersBase>& TraitsOmplMotionPlanner::parameters() const
    {
        return m_parameters;
    }

    const std::shared_ptr<TraitsEnvironmentBase>& TraitsOmplMotionPlanner::environment() const
    {
        return m_environment;
    }

    unsigned int TraitsOmplMotionPlanner::numMotionPlans() const
    {
        return m_memoization.size();
    }

}  // namespace traits
