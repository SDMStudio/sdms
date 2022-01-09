#include <sdm/algorithms/bayesian_game_solver.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/joint.hpp>



sdm::TwoPlayersBayesianGameSolver::TwoPlayersBayesianGameSolver(std::shared_ptr <BayesianGameInterface> _game, int _playerIndex) : Algorithm("TwoPlayersBayesianGameSolver") {
    game = _game;
    playerIndex = _playerIndex;
}

void sdm::TwoPlayersBayesianGameSolver::initialize(){
    env = IloEnv();
    model = env;
    vars = env;
    bool loadedLP = getLPFromBayesianGame();
    if (!loadedLP) {
        std::cerr << "Coul not load game LP in bayesian game solver" << std::endl;
    }
}

bool sdm::TwoPlayersBayesianGameSolver::getLPFromBayesianGame(){

    try {

        std::shared_ptr<MultiDiscreteSpace> typeSpace = game->getTypeSpace()->toMultiDiscreteSpace();
        std::shared_ptr<MultiDiscreteSpace> actionSpace = game->getActionSpace()->toMultiDiscreteSpace();

        auto actionsOpPlayer = actionSpace->getSpace(abs(playerIndex-1));
        auto actionsPlayer = actionSpace->getSpace(playerIndex);
        auto typesOpPlayer = typeSpace->getSpace(abs(playerIndex-1));
        auto typesPlayer = typeSpace->getSpace(playerIndex);



        /* CONSTRUCT LP */

        // add alpha_i variables
        IloExpr obj(env);

        int inserted = 0;
        for (const auto &opType: *typesOpPlayer)
        {
            string varName = vn.getVarNameState(opType->toState());
            vars.add(IloNumVar(env, -IloInfinity, IloInfinity, ILOFLOAT, varName.c_str()));
            obj += vars[inserted];
            vn.setNumber(varName, inserted);
            inserted++;
        }
        // add objective to model
        model.add(IloMaximize(env,obj));
        obj.end();

        // add types conditional probas & proba equals to 1 constraints
        for (const auto &type: *typesPlayer)
        {
            IloExpr probaSum(env); 
            for (const auto &action: *actionsPlayer)
            {
                string varName = vn.getVarNameStateAction(type->toState(), action->toAction());
                vars.add(IloNumVar(env, 0, 1, ILOFLOAT, varName.c_str()));
                probaSum += vars[inserted];
                vn.setNumber(varName, inserted);
                inserted++;
            }
            model.add(IloRange(env, 1, probaSum, 1));
            probaSum.end();
        }

        // write game constraints

        for (const auto &opType: *typesOpPlayer)
        {

            for (const auto &opAction: *actionsOpPlayer)
            {
                IloExpr actionConstraint(env);
                for (const auto &plType: *typesPlayer)
                {
                    Joint<std::shared_ptr<Item>> jointType(std::vector<std::shared_ptr<Item>>{opType, plType});
                    if (playerIndex == 0) jointType = Joint<std::shared_ptr<Item>>(std::vector<std::shared_ptr<Item>>{plType, opType->toState()});
                    auto jointState = typeSpace->getItemAddress(jointType)->toState(); // cast à vérifier
                    double jointTypeProba = game->getJointTypesProba(jointState);
                    for (const auto &plAction: *actionsPlayer)
                    {

                        Joint<std::shared_ptr<Item>> jAction(std::vector<std::shared_ptr<Item>>{opAction, plAction});
                        if (playerIndex == 0) jAction = Joint<std::shared_ptr<Item>>(std::vector<std::shared_ptr<Item>>{plAction, opAction});
                        auto jointAction = actionSpace->getItemAddress(jAction)->toAction();

                        double coef = jointTypeProba * game->getPayoff(jointState, jointAction, playerIndex);
                        string varName = vn.getVarNameStateAction(plType->toState(), plAction->toAction());

                        actionConstraint += coef * vars[vn.getNumber(varName)];
                    }
                }
                int optiIndex = vn.getNumber(vn.getVarNameState(opType->toState()));
                actionConstraint -= vars[optiIndex];
                model.add(IloRange(env, 0, actionConstraint, IloInfinity));
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

    //save solution
    // StochasticDecisionRule sdr = StochasticDecisionRule();
    // int nTypes = game->getTypesNumbers()[playerIndex];
    // int opposingTypes = game->getTypesNumbers()[abs(playerIndex-1)];
    // int plActions = game->getGameDimensions()[playerIndex];
    // for (int i = 0; i < nTypes; i ++)
    // {
    //     for (int j = 0; j < plActions; j ++){
    //         sdr.setProbability(std::make_shared<DiscreteState>(DiscreteState(i)), std::make_shared<DiscreteAction>(DiscreteAction(j)), vals[opposingTypes + (i)*plActions + j]);
    //     }
    // }
    // solution = std::make_shared<StochasticDecisionRule>(sdr);

    terminate(); 
}

void sdm::TwoPlayersBayesianGameSolver::terminate()
{
    env.end();
}

std::string sdm::TwoPlayersBayesianGameSolver::getAlgorithmName() {
    return "TwoPlayersBayesianGameSolver";
}

std::shared_ptr<sdm::StochasticDecisionRule> sdm::TwoPlayersBayesianGameSolver::getSolution(){
    return solution;
}

void sdm::TwoPlayersBayesianGameSolver::exportLP(const char *filename)
{
    IloCplex cplex(model);
    try{
        cplex.exportModel(filename);
    }catch (IloException& e) {
        cerr << "Concert exception caught in export model: " << e << endl;
    }
}

void sdm::TwoPlayersBayesianGameSolver::test() {
    throw sdm::exception::NotImplementedException();
}

void sdm::TwoPlayersBayesianGameSolver::save() {
    throw sdm::exception::NotImplementedException();
}