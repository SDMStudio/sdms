#pragma once

#include <vector>
#include <string>
#include <sdm/world/bayesian_game_interface.hpp>
#include <sdm/types.hpp>


namespace sdm
{
    class TwoPlayersNormalFormGame : public BayesianGameInterface
    {
        public:

        TwoPlayersNormalFormGame();

        sdm::number getNumAgents() const;

        void setGameDimensions(std::vector<std::string> strMatrixDimensions);

        void addPayoffLine(std::vector<std::string> strPayoffs);

        float getJointTypesProba(std::vector<std::shared_ptr<State>> types);

        float getPayoff(std::vector<std::shared_ptr<State>> types, std::vector<std::shared_ptr<Action>> actions, int whichPlayer);
            
        std::vector<int> getGameDimensions();

        std::vector<int> getTypesNumbers();

        protected:
        
        std::vector<std::vector<double>> jointTypeProbabilities;
        std::vector<int> gameDimensions;
        std::vector<int> typesNumbers;
        std::vector<std::vector<double>> payoffMatrixes;


    };
};