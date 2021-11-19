#pragma once

#include <vector>
#include <string>
#include <sdm/world/bayesian_game_interface.hpp>


namespace sdm
{
    class TwoPlayersNormalFormGame : public BayesianGameInterface
    {
        public:

        TwoPlayersNormalFormGame();

        int getNombreAgents();

        void setGameDimensions(std::vector<std::string> strMatrixDimensions);

        void addPayoffLine(std::vector<std::string> strPayoffs);

        float getJointTypesProba(std::vector<int> types);

        float getPayoff(std::vector<int> types, std::vector<int> actions, int whichPlayer);
            
        std::vector<int> getGameDimensions();

        std::vector<int> getTypesNumbers();

        std::vector<std::vector<float>> jointTypeProbabilities;

        protected:

        std::vector<int> gameDimensions;
        std::vector<int> typesNumbers;
        std::vector<std::vector<float>> payoffMatrixes;


    };
};