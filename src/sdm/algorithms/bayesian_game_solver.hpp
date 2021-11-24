#pragma once

#include <sdm/world/bayesian_game_interface.hpp>
#include <sdm/core/action/stochastic_decision_rule.hpp>
#include <ilcplex/ilocplex.h>
#include <sdm/public/algorithm.hpp>
#include <memory>

ILOSTLBEGIN

namespace sdm
{
    class TwoPlayersBayesianGameSolver : public Algorithm
    {
        public: 

        TwoPlayersBayesianGameSolver(std::shared_ptr<BayesianGameInterface> _game, int playerIndex);
    
        void initialize(); 
    
        bool getLPFromBayesianGame(std::shared_ptr<BayesianGameInterface> game, int playerIndex);
    
        void solve();

        void test();
        
        void save();

        void terminate();

        std::string getAlgorithmName();

        std::shared_ptr<StochasticDecisionRule> getSolution();
            
        protected:
    
        IloEnv env; 
        IloModel model; 
        IloNumVarArray vars;
        std::shared_ptr<BayesianGameInterface> game;
        int playerIndex;
        std::shared_ptr<StochasticDecisionRule> solution;
    };
};