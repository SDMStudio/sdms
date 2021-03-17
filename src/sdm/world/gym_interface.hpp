/**
 * @file gym_interface.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 04/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>
#include <tuple>
#include <sdm/types.hpp>
#include <sdm/public/world.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{
    /**
     * @brief The interface that must be implemented by reinforcement learning environment. The interface is inspired by [OpenAI Gym environments](https://gym.openai.com/).
     * 
     * @tparam TObsSpace type of observation space 
     * @tparam TActSpace type of action space
     */
    template <typename TObsSpace = Space, typename TActSpace = Space, bool is_multi_agent = false>
    class GymInterface
    {
    protected:
        using observation_type = typename TObsSpace::value_type;
        using action_type = typename TActSpace::value_type;

        std::shared_ptr<TObsSpace> observation_space_;
        std::shared_ptr<TActSpace> action_space_;

    public:
        GymInterface();
        GymInterface(std::shared_ptr<TObsSpace>, std::shared_ptr<TActSpace>);
        // GymInterface(TObsSpace, TActSpace);

        std::shared_ptr<TObsSpace> getObsSpace() const;
        std::shared_ptr<TActSpace> getActionSpace() const;

        virtual observation_type reset() = 0;
        std::tuple<observation_type, std::vector<double>, bool> step(action_type) {}

        // template <bool TBool = is_multi_agent>
        // std::enable_if_t<TBool, std::tuple<observation_type, std::vector<double>, bool>>
        // step(action_type a);

        // template <bool TBool = is_multi_agent>
        // std::enable_if_t<!TBool, std::tuple<observation_type, double, bool>>
        // step(action_type a);
    };
} // namespace sdm

#include <sdm/world/gym_interface.tpp>
