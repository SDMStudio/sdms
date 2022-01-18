#pragma once

#include <vector>
#include <string>
#include <sdm/world/bayesian_game_interface.hpp>
#include <sdm/types.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/utils/struct/recursive_map.hpp>


namespace sdm
{
    class TwoPlayersNormalFormGame : public BayesianGameInterface
    {
        public:

        TwoPlayersNormalFormGame();

        sdm::number getNumAgents() const;

        std::shared_ptr<Space> getTypeSpace() const;

        std::vector<int> getTypesNumbers();

        void setGameDimensions(std::vector<std::string> strMatrixDimensions);

        std::vector<int> getGameDimensions();

        std::shared_ptr<Space> getActionSpace() const;

        void setPayoffs(std::vector<std::vector<std::string>> payoffElements);

        double getJointTypesProba(std::shared_ptr<State> joint_type);

        double getIndivTypeProba(std::shared_ptr<State> type, int agentId);

        float getPayoff(std::shared_ptr<State> types, std::shared_ptr<Action> actions, int idAgent);

        protected:
        
        std::vector<int> typesNumbers;
        std::shared_ptr<MultiDiscreteSpace> types;
        RecursiveMap<std::shared_ptr<State>, float> jTypeProbabilities;

        std::vector<int> gameDimensions;
        std::shared_ptr<MultiDiscreteSpace> actions;

        RecursiveMap<std::shared_ptr<State>, std::shared_ptr<Action>, int, float> payoffs;


    };
};