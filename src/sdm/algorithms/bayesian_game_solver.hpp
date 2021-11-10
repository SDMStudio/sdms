#include "sdm/world/two_players_bayesian_game.hpp"
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

namespace sdm
{
    class TwoPlayersBayesianGameSolver
    {
        public: 
    
        void initialize();
    
        bool getLPFromBayesianGame(TwoPlayersBayesianGame game, int playerIndex);
    
        bool solve();

        void terminate();
    
        protected:
    
        IloEnv env; 
        IloModel model; 
        IloNumVarArray vars;
    };
};