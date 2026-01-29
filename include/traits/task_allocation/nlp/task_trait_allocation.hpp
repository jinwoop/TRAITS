/* Modeling and Optimizing the Provisioning of Exhaustible Capabilities
 * for Simultaneous Task Allocation and Scheduling
 *
 * Author: Jinwoo Park
 *
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

// region Includes
// Global
#include <memory>
#include <vector>
// External
#include <nlohmann/json.hpp>
#include <Eigen/Core>
// Local
#include "traits/common/utilities/noncopyable.hpp"
// endregion

namespace traits
{
    // forward declarations
    class TraitsProblemInputs;
    class TraitsIncrementalTaskAllocationNode;
    /*!
     *
     * \class TaskTraitAllocation
     * \brief Container for a feasible plan of task trait distributions for an allocation
     */
    class TaskTraitAllocation: public Noncopyable
    {
    public:
        // region Special Member Functions
        //! \brief Default Constructor
        TaskTraitAllocation()                                 = default;
        TaskTraitAllocation(const TaskTraitAllocation&)     = delete;
        TaskTraitAllocation(TaskTraitAllocation&&) noexcept = default;
        ~TaskTraitAllocation()                                = default;
        TaskTraitAllocation& operator=(const TaskTraitAllocation&) = delete;
        TaskTraitAllocation& operator=(TaskTraitAllocation&) noexcept = default;
        // endregion

        /*!
         * \brief Full Constructor
         *
         * \param trait_deficiencies the total trait deficiencies of this distribution
         */
        TaskTraitAllocation(
                float trait_deficiencies,
                float trait_rate_deficiencies,
                std::vector<std::vector<std::vector<float>>>& robot_task_traits,
                std::vector<std::vector<std::vector<float>>>& robot_task_trait_rates,
                std::vector<std::vector<float>>& coalition_task_traits,
                std::vector<std::vector<float>>& coalition_task_trait_rates,
                std::vector<float> task_dynamic_durations,
                std::vector<float> task_total_durations,
                std::vector<float> task_intra_transition_durations,
                std::vector<std::vector<float>>& task_trait_durations,
                std::vector<float> result_robot_inter_task_transition_velocities,
                std::vector<float> result_robot_battery_consumptions);


        //! \returns The trait_deficiencies (or the total execution time) of the schedule
        [[nodiscard]] inline float trait_deficiencies() const;
        [[nodiscard]] inline float trait_rate_deficiencies() const;

        //! \returns A list of the robot trait for tasks
        [[nodiscard]] inline std::vector<std::vector<std::vector<float>>>& robot_task_traits();
        [[nodiscard]] inline float robot_task_traits(unsigned int task_nr,
                                                     unsigned int robot_nr,
                                                     unsigned int trait_nr);
        [[nodiscard]] inline std::vector<std::vector<std::vector<float>>>& robot_task_trait_rates();
        [[nodiscard]] inline float robot_task_trait_rates(unsigned int task_nr,
                                                          unsigned int robot_nr,
                                                          unsigned int trait_nr);

        [[nodiscard]] inline std::vector<std::vector<float>>& coalition_task_traits();
        [[nodiscard]] inline const float coalition_task_traits(unsigned int task_nr,
                                                         unsigned int trait_nr) const;

        [[nodiscard]] inline std::vector<std::vector<float>>& coalition_task_trait_rates();
        [[nodiscard]] inline const float coalition_task_trait_rates(unsigned int task_nr,
                                                              unsigned int trait_nr) const;

        [[nodiscard]] inline std::vector<float>& task_dynamic_durations();
        [[nodiscard]] inline std::vector<float>& task_durations();
        [[nodiscard]] inline float task_durations(unsigned int task_nr);
        [[nodiscard]] inline float task_dynamic_durations(unsigned int task_nr);
        [[nodiscard]] inline std::vector<std::vector<float>>& task_trait_durations();
        [[nodiscard]] inline const float task_trait_durations(unsigned int task_nr,
                                                        unsigned int trait_nr) const;

        nlohmann::json serializeToJson(
                const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
                const Eigen::MatrixXf& allocation) const;

    protected:
        float m_result_trait_deficiencies;
        float m_result_trait_rate_deficiencies;

        std::vector<float> m_task_dynamic_durations;
        std::vector<float> m_task_total_durations;
        std::vector<float> m_task_intra_transition_durations;
        std::vector<float> m_result_robot_inter_task_transition_velocities;
        std::vector<float> m_result_robot_battery_consumptions;
        std::vector<std::vector<float>> m_task_trait_durations;

        std::vector<std::vector<float>> m_coalition_task_traits;
        std::vector<std::vector<float>> m_coalition_task_trait_rates;

        std::vector<std::vector<std::vector<float>>> m_robot_task_traits;
        std::vector<std::vector<std::vector<float>>> m_robot_task_trait_rates;

    };

    // Inline Functions
    float TaskTraitAllocation::trait_deficiencies() const
    {
        return m_result_trait_deficiencies;
    }

    float TaskTraitAllocation::trait_rate_deficiencies() const
    {
        return m_result_trait_rate_deficiencies;
    }

    std::vector<std::vector<std::vector<float>>>& TaskTraitAllocation::robot_task_traits()
    {
        return m_robot_task_traits;
    }

    float TaskTraitAllocation::robot_task_traits(unsigned int task_nr,
                                                 unsigned int robot_nr,
                                                 unsigned int trait_nr)
    {
        return m_robot_task_traits[task_nr][robot_nr][trait_nr];
    }

    std::vector<std::vector<std::vector<float>>>& TaskTraitAllocation::robot_task_trait_rates()
    {
        return m_robot_task_trait_rates;
    }

    float TaskTraitAllocation::robot_task_trait_rates(unsigned int task_nr,
                                                      unsigned int robot_nr,
                                                      unsigned int trait_nr)
    {
        return m_robot_task_trait_rates[task_nr][robot_nr][trait_nr];
    }

    std::vector<std::vector<float>>& TaskTraitAllocation::coalition_task_traits(){
        return m_coalition_task_traits;
    }

    const float TaskTraitAllocation::coalition_task_traits(unsigned int task_nr,
                                                     unsigned int trait_nr) const
    {
        return m_coalition_task_traits[task_nr][trait_nr];
    }

    std::vector<std::vector<float>>& TaskTraitAllocation::coalition_task_trait_rates(){
        return m_coalition_task_trait_rates;
    }

    const float TaskTraitAllocation::coalition_task_trait_rates(unsigned int task_nr,
                                                          unsigned int trait_nr) const
    {
        return m_coalition_task_trait_rates[task_nr][trait_nr];
    }

    std::vector<float>& TaskTraitAllocation::task_durations()
    {
        return m_task_total_durations;
    }

    float TaskTraitAllocation::task_durations(unsigned int task_nr)
    {
        return m_task_total_durations[task_nr];
    }

    std::vector<float>& TaskTraitAllocation::task_dynamic_durations()
    {
        return m_task_dynamic_durations;
    }

    float TaskTraitAllocation::task_dynamic_durations(unsigned int task_nr)
    {
        return m_task_dynamic_durations[task_nr];
    }

    std::vector<std::vector<float>>& TaskTraitAllocation::task_trait_durations()
    {
        return m_task_trait_durations;
    }

    const float TaskTraitAllocation::task_trait_durations(unsigned int task_nr, unsigned int trait_nr) const
    {
        return m_task_trait_durations[task_nr][trait_nr];
    }

}  // namespace traits
