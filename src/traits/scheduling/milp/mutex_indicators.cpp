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
#include "traits/scheduling/milp/mutex_indicators.hpp"

// External
#include <fmt/format.h>
#include <fmt/ranges.h>
// Local
#include "traits/common/utilities/error.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/scheduling/milp/ms_name_scheme_base.hpp"

namespace traits
{
    MutexIndicators::MutexIndicators(const std::set<std::pair<unsigned int, unsigned int>>& mutex_constraints,
                                     const std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints,
                                     const std::shared_ptr<const MsNameSchemeBase>& name_scheme,
                                     bool isSearch)
        : m_precedence_constraints(precedence_constraints)
        , m_name_scheme(name_scheme)
        , m_search(isSearch)
    {
        for(const auto& p: mutex_constraints)
        {
            // precedence constraints overrides.
            if(m_search &&
                ( m_precedence_constraints.contains(p) || m_precedence_constraints.contains({p.second, p.first}) )
              )
            {
                continue;
            }

            // Give an empty GRBVar for now
            m_indicators[p] = GRBVar();
            if (!m_search)
            {
                m_indicators[{p.second, p.first}] = GRBVar();
            }
        }
    }

    MutexIndicators::MutexIndicators(const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs,
                                     const std::shared_ptr<const MsNameSchemeBase>& name_scheme,
                                     bool isSearch)
            : MutexIndicators(problem_inputs->mutexConstraints(),
                              problem_inputs->precedenceConstraints(),
                              name_scheme,
                              isSearch)
    {}

    void MutexIndicators::createVariables(GRBModel& model)
    {
        // Create mutex indicators - binary
        for(auto& [k, v]: m_indicators)
        {
            v = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, m_name_scheme->createMutexIndicatorName(k.first, k.second));
        }
    }

    std::vector<std::pair<unsigned int, unsigned int>> MutexIndicators::precedenceSet() const
    {
        std::vector<std::pair<unsigned int, unsigned int>> rv;
        rv.reserve(m_indicators.size());
        for(const auto& [p, indicator]: m_indicators)
        {
            // NOTE: To avoid indicator != 1.0 b/c 0.9999999
            if(indicator.get(GRB_DoubleAttr_X) > 0.5)
            {
                rv.emplace_back(p);
            }
            else
            {
                // reverse the order
                rv.emplace_back(p.second, p.first);
            }
        }
        return rv;
    }

    GRBVar& MutexIndicators::get(const std::pair<unsigned int, unsigned int>& p)
    {
        if(m_indicators.contains(p))
        {
            return m_indicators[p];
        }
        throw createLogicError(fmt::format("Cannot find indicator for {}", p));
    }
}  // namespace traits