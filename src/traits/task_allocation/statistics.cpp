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
#include "traits/task_allocation/statistics.hpp"

// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/time_keeper.hpp"
#include "traits/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/scheduling/milp/milp_scheduler.hpp"

namespace traits
{
    Statistics::Statistics(const std::string& timer_name)
        : SearchStatisticsCommon(timer_name)
    {}

    nlohmann::json Statistics::serializeToJson(const std::shared_ptr<const ProblemInputs>& problem_inputs) const
    {
        const auto& traits_problem_inputs = std::dynamic_pointer_cast<const TraitsProblemInputs>(problem_inputs);

        nlohmann::json j = SearchStatisticsCommon::serializeToJson(problem_inputs);

        const float motion_planning_time = TimeKeeper::instance().time(constants::k_motion_planning_time);
        const float scheduling_time      = TimeKeeper::instance().time(constants::k_scheduling_time);

        j[constants::k_total_time] = TimeKeeper::instance().time(constants::k_total_time);
        j[constants::k_scheduling_time]      = scheduling_time;
        j[constants::k_motion_planning_time] = motion_planning_time;
        unsigned int num_motion_plans        = 0;
        for(const std::shared_ptr<TraitsOmplMotionPlanner>& motion_planner: traits_problem_inputs->motionPlanners())
        {
            num_motion_plans += motion_planner->numMotionPlans();
        }
        j[constants::k_num_motion_plans]         = num_motion_plans;
        j[constants::k_num_motion_plan_failures] = TraitsOmplMotionPlanner::numFailures();
        j[constants::k_num_scheduling_failures]  = TraitsSchedulerBase::numFailures();
        j[constants::k_num_scheduling_iterations] = TraitsMilpScheduler::numIterations();

        return j;
    }

}  // namespace traits