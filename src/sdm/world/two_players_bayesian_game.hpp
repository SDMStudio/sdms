#pragma once

#include <vector>
#include <string>
#include <sdm/world/bayesian_game_interface.hpp>

#ifndef TWOPLAYERSBAYESIANGAME

namespace sdm
{
    class TwoPlayersBayesianGame : public BayesianGameInterface
    {
        public:

        TwoPlayersBayesianGame();

        int getNombreAgents();

        void setTypeNumbers(std::vector<std::string> strTypes);

        void setGameDimensions(std::vector<std::string> strMatrixDimensions);

        void addJointTypeProbabilities(std::vector<std::string> strProbabilities);

        void addPayoffLine(std::vector<std::string> strPayoffs);

        float getJointTypesProba(std::vector<int> types);

        float getPayoff(std::vector<int> types, std::vector<int> actions, int whichPlayer);
            
        std::vector<int> getGameDimensions();

        std::vector<int> getTypesNumbers();

        protected:
        
        std::vector<std::vector<float>> jointTypeProbabilities;
        std::vector<int> gameDimensions;
        std::vector<int> typesNumbers;
        std::vector<std::vector<float>> payoffMatrixes;


    };
};

#define TWOPLAYERSBAYESIANGAME
#endif