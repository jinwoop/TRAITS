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
// Global
#include <fstream>
#include <filesystem>
#include <memory>
// External
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
// Project
#include <traits/config.hpp>
#include <traits/problem_inputs/traits_problem_inputs.hpp>
#include <traits/common/utilities/logger.hpp>
#include <traits/task_allocation/traits.hpp>
#include <traits/task_allocation/dynamic_time_extended_task_allocation_quality.hpp>

namespace traits::unittests
{
#ifndef NO_MILP
    /*!
     * TRAITS Sample Scenario
     */
    TEST(TRAITS, simple_run)
    {
        {
            std::string file_path = std::string(s_data_dir) + std::string("/problem_inputs/traits/sample_input.json");
            std::ifstream fin(file_path);
            nlohmann::json j;
            fin >> j;
            TimerRunner timer_runner(constants::k_total_time);
            std::shared_ptr<const TraitsProblemInputs> problem_inputs = j.get<std::shared_ptr<TraitsProblemInputs>>();

            TRAITS traits(problem_inputs);

            SearchResults<TraitsIncrementalTaskAllocationNode, Statistics> results = traits.search();
            ASSERT_TRUE(results.foundGoal());
            std::shared_ptr<Statistics> statistics = results.statistics();
            ASSERT_TRUE(statistics);
            fmt::print("time: {}s\n", TimeKeeper::instance().time("traits"));
            results.writeToFile("../output_simple_run.json", problem_inputs);
        }
        NlpSolverBase::clearEnvironments();
        MilpSolverBase::clearEnvironments();
    }

#endif
}  // namespace traits::unittests