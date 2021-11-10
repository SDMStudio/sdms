#include "sdm/algorithms/bayesian_game_solver.hpp"

void sdm::TwoPlayersBayesianGameSolver::initialize(){
    env = IloEnv();
    model = env;
    vars = env;
}

bool sdm::TwoPlayersBayesianGameSolver::getLPFromBayesianGame(TwoPlayersBayesianGame game, int playerIndex){
    try {
        vector<int> typesNumbers(game.getTypesNumbers());
        vector<int> matrixDimensions(game.getMatrixDimensions());
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
                    float jointTypeProba = game.getJointTypeProba(opType, plType);
                    if (playerIndex == 0) jointTypeProba = game.getJointTypeProba(plType, opType);
                    for(int plAction = 0; plAction < matrixDimensions[playerIndex]; plAction ++){
                        float coef;
                        if(playerIndex == 0){
                            coef = jointTypeProba * game.getPayoff(plType, opType, plAction, opAction, playerIndex);
                        }else{
                            coef = jointTypeProba * game.getPayoff(opType, plType, opAction, plAction, playerIndex);
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

    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

}

bool sdm::TwoPlayersBayesianGameSolver::solve(){
    IloCplex c(model);
    if ( !c.solve() ) {
        cout << "could not solve" << endl;
        throw(-1);
        return false;
    }
    cout << "solved" << endl;
    IloNumArray vals(env);
    env.out() << "Solution status = " << c.getStatus() << endl;
    env.out() << "Solution value = " << c.getObjValue() << endl;
    c.getValues(vals, vars);
    env.out() << "Variables = " << vars << endl;
    env.out() << "Values = " << vals << endl;
    return true;
}

void sdm::TwoPlayersBayesianGameSolver::terminate()
{
    env.end();
}