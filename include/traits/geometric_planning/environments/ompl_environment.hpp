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
#include <mutex>
// External
#include <nlohmann/json.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
// Local
#include "traits/geometric_planning/environments/environment_base.hpp"

namespace traits
{
    /*
     * https://ompl.kavrakilab.org/classompl_1_1Grid.html
     * https://ompl.kavrakilab.org/classPolyWorld.html
     */

    // Forward Declarations
    enum class OmplStateSpaceType : uint8_t;
    enum class OmplEnvironmentType : uint8_t;

    /*!
     * Abstract base class for the environment that is to be used with motion planners from OMPL
     *
     * Derivative classes are used to setupData the state space and determine if states are valid
     */
    class TraitsOmplEnvironment
            : public ompl::base::StateValidityChecker
                    , public TraitsEnvironmentBase
    {
    public:
        //! Initializes the factory
        static void init();

        //! \copydoc ompl::base::StateValidityChecker
        [[nodiscard]] virtual bool isValid(const ompl::base::State* state) const override = 0;

        //! \returns The state space for this environment
        [[nodiscard]] inline const std::shared_ptr<ompl::base::StateSpace>& stateSpace() const;

        //! \returns The type of ompl environment represented
        [[nodiscard]] inline OmplEnvironmentType omplEnvironmentType() const;

        //! \returns The type of state space represented
        [[nodiscard]] inline OmplStateSpaceType stateSpaceType() const;

    protected:
        //! Default Constructor
        TraitsOmplEnvironment(OmplEnvironmentType environment_type, OmplStateSpaceType state_space_type);

        OmplEnvironmentType m_environment_type;
        OmplStateSpaceType m_state_space_type;
        std::shared_ptr<ompl::base::StateSpace> m_state_space;
    };

    // Inline Functions
    const std::shared_ptr<ompl::base::StateSpace>& TraitsOmplEnvironment::stateSpace() const
    {
        assert(m_state_space);
        return m_state_space;
    }

    OmplEnvironmentType TraitsOmplEnvironment::omplEnvironmentType() const
    {
        return m_environment_type;
    }

    OmplStateSpaceType TraitsOmplEnvironment::stateSpaceType() const
    {
        return m_state_space_type;
    }
}  // namespace traits

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<traits::TraitsOmplEnvironment>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<traits::TraitsOmplEnvironment> from_json(const json& j);
    };
}  // namespace nlohmann
