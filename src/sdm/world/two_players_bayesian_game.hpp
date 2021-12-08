#pragma once

#include <vector>
#include <string>
#include <sdm/world/bayesian_game_interface.hpp>
#include <sdm/types.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/utils/struct/recursive_map.hpp>


#ifndef TWOPLAYERSBAYESIANGAME

namespace sdm
{
    class TwoPlayersBayesianGame : public BayesianGameInterface
    {
        public:

        TwoPlayersBayesianGame();

        number getNumAgents() const;
        
        std::shared_ptr<Space> getTypeSpace() const;

        std::vector<int> getTypesNumbers();

        void setTypeNumbers(std::vector<std::string> strTypes);

        void setGameDimensions(std::vector<std::string> strMatrixDimensions);

        std::vector<int> getGameDimensions();

        std::shared_ptr<Space> getActionSpace() const;

        void setJointTypeProbabilities(std::vector<std::vector<std::string>> probabilities);

        void setPayoffs(std::vector<std::vector<std::string>> payoffElements);

        double getJointTypesProba(std::shared_ptr<State> joint_type);

        float getPayoff(std::shared_ptr<State> types, std::shared_ptr<Action> actions, int idAgent);

        protected:

        RecursiveMap<std::shared_ptr<State>, float> jTypeProbabilities;
        std::shared_ptr<MultiDiscreteSpace> types;
        std::vector<int> typesNumbers;

        std::vector<int> gameDimensions;
        std::shared_ptr<MultiDiscreteSpace> actions;

        RecursiveMap<std::shared_ptr<State>, std::shared_ptr<Action>, int, float> payoffs;

    };
};

#define TWOPLAYERSBAYESIANGAME
#endif