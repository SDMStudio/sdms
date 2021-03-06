#ifdef WITH_CPLEX

#include "sdm/algorithms/bayesian_game_solver.hpp"
#include <sdm/exception.hpp>


sdm::TwoPlayersBayesianGameSolver::TwoPlayersBayesianGameSolver(std::shared_ptr <BayesianGameInterface> _game, int _playerIndex) : Algorithm("TwoPlayersBayesianGameSolver") {
    game = _game;
    playerIndex = _playerIndex;
}

void sdm::TwoPlayersBayesianGameSolver::initialize(){
    env = IloEnv();
    model = env;
    vars = env;
    bool loadedLP = getLPFromBayesianGame(game, playerIndex);
    if (!loadedLP) {
        std::cerr << "Coul not load game LP in bayesian game solver" << std::endl;
    }
}

bool sdm::TwoPlayersBayesianGameSolver::getLPFromBayesianGame(std::shared_ptr<BayesianGameInterface> game, int playerIndex){
    try {
        std::vector<int> typesNumbers(game->getTypesNumbers());
        std::vector<int> matrixDimensions(game->getGameDimensions());

        /* CONSTRUCT LP */

        // add alpha_i variables
        int numberOfOptiVars = typesNumbers[abs(playerIndex - 1)];
        IloExpr obj(env);
        for (int i = 0; i < numberOfOptiVars; i ++){
            string varName = "alpha" + to_string(i);
            vars.add(IloNumVar(env, -IloInfinity, IloInfinity, ILOFLOAT, varName.c_str()));
            obj += vars[i];
        }

        // add objective to model
        model.add(IloMaximize(env,obj));
        obj.end();

        // add types conditional probas & proba equals to 1 constraints
        for (int t = 0; t < typesNumbers[playerIndex]; t ++){
            IloExpr probaSum(env); 
            for (int a = 0; a < matrixDimensions[playerIndex]; a++){
                string varName = "p" + to_string(t) + to_string(a);

                vars.add(IloNumVar(env, 0, 1, ILOFLOAT, varName.c_str()));
                probaSum += vars[numberOfOptiVars + t*matrixDimensions[playerIndex] + a];
            }
            model.add(IloRange(env, 1, probaSum, 1));
            probaSum.end();
        }

        // write game constraints
        int opposingPlayerTypes = typesNumbers[abs(playerIndex -1)];
        for (int opType = 0; opType < opposingPlayerTypes; opType++){
            for (int opAction = 0; opAction < matrixDimensions[abs(playerIndex -1)]; opAction ++){
                IloExpr actionConstraint(env);
                for (int plType = 0; plType < typesNumbers[playerIndex]; plType ++){
                    float jointTypeProba = game->getJointTypesProba(std::vector<int>{opType, plType});
                    if (playerIndex == 0) jointTypeProba = game->getJointTypesProba(std::vector<int>{plType, opType});
                    for(int plAction = 0; plAction < matrixDimensions[playerIndex]; plAction ++){
                        float coef;
                        if(playerIndex == 0){
                            coef = jointTypeProba * game->getPayoff(std::vector<int>{plType, opType}, std::vector<int>{plAction, opAction}, playerIndex);
                        }else{
                            coef = jointTypeProba * game->getPayoff(std::vector<int>{opType, plType}, std::vector<int>{opAction, plAction}, playerIndex);
                        }
                        actionConstraint += coef*vars[numberOfOptiVars + plType*matrixDimensions[0] + plAction];
                    }
                }
                actionConstraint -= vars[opType];
                string constraintName = "c" + to_string(opType) + to_string(opAction);
                model.add(IloRange(env, 0, actionConstraint, IloInfinity, constraintName.c_str()));
                actionConstraint.end();
            }
        }
        return true;
    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    return false;
}

void sdm::TwoPlayersBayesianGameSolver::solve(){
    IloCplex c(model);
    if ( !c.solve() ) {
        cout << "could not solve" << endl;
        throw(-1);
    }
    IloNumArray vals(env);
    env.out() << "Solution status = " << c.getStatus() << endl;
    env.out() << "Solution value = " << c.getObjValue() << endl;
    c.getValues(vals, vars);
    env.out() << "Variables = " << vars << endl;
    env.out() << "Values = " << vals << endl;
    terminate(); 
}

void sdm::TwoPlayersBayesianGameSolver::terminate()
{
    env.end();
}

std::string sdm::TwoPlayersBayesianGameSolver::getAlgorithmName() {
    return "TwoPlayersBayesianGameSolver";
}

void sdm::TwoPlayersBayesianGameSolver::test() {
    throw sdm::exception::NotImplementedException();
}

void sdm::TwoPlayersBayesianGameSolver::save() {
    throw sdm::exception::NotImplementedException();
}
#endif