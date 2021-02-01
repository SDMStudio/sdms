/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/public/world.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

//!
//! \file     gym_interface.hpp
//! \author   David Albert
//! \brief    GymInterface class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This abstract class provide an interface based on gym environment.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  GymInterface
    //! \brief Provides a Gym like interface 
    template <typename TObsSpace = Space, typename TActSpace = Space>
    class GymInterface : public World
    {
    protected:
        using observation_type = typename TObsSpace::value_type;
        using action_type = typename TActSpace::value_type;

        std::shared_ptr<TObsSpace> observation_space_;
        std::shared_ptr<TActSpace> action_space_;
    public:
        GymInterface(std::shared_ptr<TObsSpace> , std::shared_ptr<TActSpace> );
        // GymInterface(TObsSpace, TActSpace);

        std::shared_ptr<TObsSpace> getObsSpace() const;
        std::shared_ptr<TActSpace> getActionSpace() const;

        virtual observation_type reset() = 0;
        virtual std::tuple<observation_type, std::vector<double>, bool> step(action_type a) = 0; // std::tuple<Observation, Reward, bool, map>
    };
} // namespace sdm
