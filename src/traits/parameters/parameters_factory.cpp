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
#include "traits/parameters/parameters_factory.hpp"

// region Includes
// Local
#include "traits/common/utilities/error.hpp"
#include "traits/parameters/motion_planner_parameters_factory.hpp"
#include "traits/parameters/scheduler_parameters_factory.hpp"
#include "traits/parameters/search_parameters_factory.hpp"
#include "traits/parameters/trait_distribution_parameters_factory.hpp"
// endregion

namespace traits
{
    ParametersFactory& ParametersFactory::instance()
    {
        static ParametersFactory singleton;
        return singleton;
    }

    std::shared_ptr<ParametersBase> ParametersFactory::create(ParametersFactory::Type t, const nlohmann::json& j) const
    {
        switch(t)
        {
            case Type::e_motion_planner:
            {
                return m_motion_planner_parameters_factory.create(j);
            }
            case Type::e_scheduler:
            {
                return m_scheduler_parameters_factory.create(j);
            }
            case Type::e_trait_distribution:
            {
                return m_trait_distribution_parameters_factory.create(j);
            }
            case Type::e_search:
            {
                return m_search_parameters_factory.create(j);
            }
            default:
            {
                throw createLogicError("Unknown type of algorithm");
            }
        }
    }

    ParametersFactory::ParametersFactory()
            : m_motion_planner_parameters_factory(MotionPlannerParametersFactory::instance())
            , m_scheduler_parameters_factory(SchedulerParametersFactory::instance())
            , m_trait_distribution_parameters_factory(TraitDistributionParametersFactory::instance())
            , m_search_parameters_factory(SearchParametersFactory::instance())
    {}
}  // namespace traits
