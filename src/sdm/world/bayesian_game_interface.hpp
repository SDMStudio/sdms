#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm {

    class BayesianGameInterface {
        
        public:

            virtual ~BayesianGameInterface(){}

            virtual number getNumAgents() const = 0;

            virtual std::vector<int> getGameDimensions() = 0;

            virtual std::vector<int> getTypesNumbers() = 0;

            virtual float getPayoff(std::vector<std::shared_ptr<State>> types, std::vector<std::shared_ptr<Action>> actions, int idAgent) = 0;

            virtual float getJointTypesProba(std::vector<std::shared_ptr<State>> types) = 0;

    };
}