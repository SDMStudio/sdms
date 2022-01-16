#pragma once

#include <sdm/world/bayesian_game_interface.hpp>
#include <sdm/core/action/stochastic_decision_rule.hpp>
#include <ilcplex/ilocplex.h>
#include <sdm/public/algorithm.hpp>
#include <memory>
#include <sdm/utils/linear_programming/variable_naming.hpp>

ILOSTLBEGIN

namespace sdm
{
    class HS4BG : public Algorithm
    {
        public: 

        HS4BG(std::shared_ptr<BayesianGameInterface> _game);
    
        void initialize(); 
        
        void solve();

        void test();
        
        void save();

        void terminate();

        void saveSolution();

        std::string getAlgorithmName();

        std::vector<std::shared_ptr<StochasticDecisionRule>> getSolution();
            
        protected:

        void initLP();

        double getActionProbability(std::shared_ptr<State> type, std::shared_ptr<Action> action, int agentId, bool fromInitialDistribution);

        std::shared_ptr<State> naiveHS(int agentId);

        std::shared_ptr<Action> bestResponse(std::shared_ptr<State> type, int agentId, int step);

        void updateLP(std::shared_ptr<StochasticDecisionRule> pi1, std::shared_ptr<StochasticDecisionRule> pi2);
    
        void updateStrategies();
        
        IloEnv env; 
        IloModel model1; 
        IloNumVarArray vars1;
        IloModel model2; 
        IloNumVarArray vars2;
        std::shared_ptr<BayesianGameInterface> game;
        std::shared_ptr<StochasticDecisionRule> strategy1;
        std::shared_ptr<StochasticDecisionRule> strategy2;
        VarNaming vn1;
        VarNaming vn2;
    };
};