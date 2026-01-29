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
#include <string>
// External
#include <eigen3/Eigen/Core>
#include <nlohmann/json.hpp>

namespace traits
{
    // Forward Declarations
    class TraitsOmplMotionPlanner;
    class ConfigurationBase;

    /*!
     * A container for the information associated with a species of robots
     */
    class Species
    {
    public:
        //! Default constructor for json
        Species();

        //! Constructor
        Species(const std::string& name,
                const Eigen::VectorXf& traits_max,
                const Eigen::VectorXf& trait_rates_max,
                const Eigen::VectorXd& traits_provisionable,
                const Eigen::VectorXd& traits_exhaustible,
                const Eigen::VectorXf& current_trait_functions_coeff,
                const Eigen::VectorXf& current_trait_rate_functions_coeff,
                const double max_battery_capacity,
                const float peukert_coeff,
                const float idle_current,
                const float max_possible_current,
                const float radius,
                const float speed,
                const float speed_coeff,
                const std::shared_ptr<TraitsOmplMotionPlanner>& motion_planner);


        //! Destructor
        virtual ~Species();

        //! \returns The name of the Species
        [[nodiscard]] inline const std::string& name() const;

        //! \returns An identifier for the Species
        [[nodiscard]] inline unsigned int id() const;

        //! \returns The initial values of traits for the species
        [[nodiscard]] virtual inline const Eigen::VectorXf& traits_max() const;

        //! \returns The time decaying rate of traits for the species
        [[nodiscard]] virtual inline const Eigen::VectorXf& trait_rates_max() const;

        [[nodiscard]] virtual inline const Eigen::VectorXf& current_trait_functions_coeff() const;
        [[nodiscard]] virtual inline const Eigen::VectorXf& current_trait_rate_functions_coeff() const;

        //! \returns Whether the traits are aggregatable for the species
        [[nodiscard]] virtual inline const Eigen::VectorXd& traits_provisionable() const;
        [[nodiscard]] virtual inline const Eigen::VectorXd& traits_exhaustible() const;

        //! \returns The maximum capacity of the battery for the species
        [[nodiscard]] inline double max_battery_capacity() const;

        [[nodiscard]] inline float max_possible_current() const;

        [[nodiscard]] inline float peukert_coeff() const;
        [[nodiscard]] inline float idle_current() const;
        [[nodiscard]] inline float speed_coeff() const;

        //! \returns The radius of a bounding circle/sphere for this species of robot
        [[nodiscard]] inline float boundingRadius() const;

        //! \returns The speed for this type of robot
        [[nodiscard]] inline float max_speed() const;

        //! \returns The speed for this type of robot
        [[nodiscard]] inline float value() const;

        //! \returns The the motion planner for this species of robot
        [[nodiscard]] inline const std::shared_ptr<TraitsOmplMotionPlanner>& motionPlanner() const;

        /*!
         *  Deserializes a json object with species information
         *
         *  \note A custom function because the motion planners are needed
         */
        static std::shared_ptr<const Species> loadJson(
                const nlohmann::json& j,
                const std::vector<std::shared_ptr<TraitsOmplMotionPlanner>>& motion_planners);

        //    private:
    protected:

        unsigned int m_id;
        std::string m_name;
        Eigen::VectorXf m_traits_max;
        Eigen::VectorXf m_trait_rates_max;

        Eigen::VectorXd m_traits_provisionable;
        Eigen::VectorXd m_traits_exhaustible;

        Eigen::VectorXf m_current_trait_functions_coeff;
        Eigen::VectorXf m_current_trait_rate_functions_coeff;

        double m_max_battery_capacity;
        float m_peukert_coeff;
        float m_idle_current;
        float m_speed_coeff;
        float m_max_possible_current;
        float m_bounding_radius;
        float m_speed;
        std::shared_ptr<TraitsOmplMotionPlanner> m_motion_planner;

        static unsigned int s_num_species;
        static unsigned int s_next_id;
    };

    // Inline Functions
    const std::string& Species::name() const
    {
        return m_name;
    }

    unsigned int Species::id() const
    {
        return m_id;
    }

    const Eigen::VectorXf& Species::traits_max() const
    {
        return m_traits_max;
    }

    const Eigen::VectorXf& Species::trait_rates_max() const
    {
        return m_trait_rates_max;
    }

    const Eigen::VectorXf& Species::current_trait_functions_coeff() const
    {
        return m_current_trait_functions_coeff;
    }

    const Eigen::VectorXf& Species::current_trait_rate_functions_coeff() const
    {
        return m_current_trait_rate_functions_coeff;
    }

    const Eigen::VectorXd& Species::traits_provisionable() const {
        return m_traits_provisionable;
    }

    const Eigen::VectorXd& Species::traits_exhaustible() const {
        return m_traits_exhaustible;
    }

    double Species::max_battery_capacity() const
    {
        return m_max_battery_capacity;
    }

    float Species::max_possible_current() const
    {
         return m_max_possible_current;
    }

    float Species::peukert_coeff() const
    {
        return m_peukert_coeff;
    }

    float Species::idle_current() const
    {
        return m_idle_current;
    }

    float Species::speed_coeff() const
    {
        return m_speed_coeff;
    }

    float Species::boundingRadius() const
    {
        return m_bounding_radius;
    }

    float Species::max_speed() const
    {
        return m_speed;
    }

    const std::shared_ptr<TraitsOmplMotionPlanner>& Species::motionPlanner() const
    {
        return m_motion_planner;
    }
}  // namespace traits
