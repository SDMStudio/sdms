#pragma once

#include <vector>

namespace sdm {

    class BayesianGameInterface {
        
        public:            
            virtual int getNombreAgents() = 0; // english getNumAgents

            virtual std::vector<int> getGameDimensions() = 0;

            virtual std::vector<int> getTypesNumbers() = 0;

            virtual float getPayoff(std::vector<int> types, std::vector<int> actions, int idAgent) = 0;

            virtual float getJointTypesProba(std::vector<int> types) = 0;

    };
}