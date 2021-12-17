#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/space/space.hpp>

namespace sdm
{

    class BayesianGameInterface
    {

    public:
        virtual ~BayesianGameInterface() {}

        virtual number getNumAgents() const = 0;

        virtual std::shared_ptr<Space> getTypeSpace() const = 0;

        virtual std::shared_ptr<Space> getActionSpace() const = 0;

        virtual std::vector<int> getGameDimensions() = 0;

        virtual std::vector<int> getTypesNumbers() = 0;

        virtual float getPayoff(std::shared_ptr<State> types, std::shared_ptr<Action> actions, int idAgent) = 0;

        virtual double getJointTypesProba(std::shared_ptr<State> joint_type) = 0;
    };
}