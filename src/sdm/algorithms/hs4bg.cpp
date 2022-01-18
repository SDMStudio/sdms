#include <sdm/algorithms/hs4bg.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/joint.hpp>
#include <limits>
#include <random>


sdm::HS4BG::HS4BG(std::shared_ptr<BayesianGameInterface> _game) : Algorithm("HS4BG") {
    game = _game;
}

void sdm::HS4BG::initialize(){
    env = IloEnv();
    model1 = env;
    vars1 = env;
    model2 = env;
    vars2 = env;

    initLP();
}

void sdm::HS4BG::initLP() {
    try {
        std::shared_ptr<MultiDiscreteSpace> typeSpace = game->getTypeSpace()->toMultiDiscreteSpace();
        std::shared_ptr<MultiDiscreteSpace> actionSpace = game->getActionSpace()->toMultiDiscreteSpace();

        auto actions1 = actionSpace->getSpace(0);
        auto actions2 = actionSpace->getSpace(1);
        auto types1 = typeSpace->getSpace(0);
        auto types2 = typeSpace->getSpace(1);

        /* --------- ADD ALPHA_i VARS FOR BOTH LP ------------ */

        IloExpr obj(env);

        int inserted1 = 0;
        for (const auto &t2: *types2)
        {
            string varName = vn1.getVarNameState(t2->toState());
            vars1.add(IloNumVar(env, -IloInfinity, IloInfinity, ILOFLOAT, varName.c_str()));
            obj += vars1[inserted1];
            vn1.setNumber(varName, inserted1);
            inserted1++;
        }

        // add objective to model
        model1.add(IloMaximize(env,obj));
        obj.end();

        IloExpr obj2(env);
        int inserted2 = 0;
        for (const auto &t1: *types1)
        {
            string varName = vn2.getVarNameState(t1->toState());
            vars2.add(IloNumVar(env, -IloInfinity, IloInfinity, ILOFLOAT, varName.c_str()));
            obj2 += vars2[inserted2];
            vn2.setNumber(varName, inserted2);
            inserted2++;
        }
        // add objective to model
        model2.add(IloMaximize(env,obj2));
        obj2.end();

        /* --------- ADD SDR OPTI VARS AND CORRESPONDING CONSTRAINTS FOR BOTH LP ------------ */
        // TODO : initialiser dnegi en prenant la première action à chaque fois
        StochasticDecisionRule sdr1 = StochasticDecisionRule();
        DeterministicDecisionRule ddr1 = DeterministicDecisionRule();
        double distributionTotal;
        for (const auto &type: *types1)
        {
            IloExpr probaSum(env); 
            distributionTotal = 0;
            ddr1.setProbability(type->toState(), actions1->toMultiDiscreteSpace()->getItem(0)->toAction());
            for (const auto &action: *actions1)
            {
                string varName = vn1.getVarNameStateAction(type->toState(), action->toAction());
                vars1.add(IloNumVar(env, 0, 1, ILOFLOAT, varName.c_str()));
                probaSum += vars1[inserted1];
                vn1.setNumber(varName, inserted1);
                inserted1++;
                // init strategy1
                double r = (double) rand() / (RAND_MAX);
                sdr1.setProbability(type->toState(), action->toAction(), r);
                distributionTotal += r;
            }
            model1.add(IloRange(env, 1, probaSum, 1));
            probaSum.end();

            //make initial distribution sum equal to 1
            for (const auto &action: *actions1){
                sdr1.setProbability(type->toState(), action->toAction(), sdr1.getProbability(type->toState(), action->toAction()) / distributionTotal);
            }
        }
        strategy1 = std::make_shared<StochasticDecisionRule>(sdr1);
        pureStrategy1 = std::make_shared<DeterministicDecisionRule>(ddr1);

        StochasticDecisionRule sdr2 = StochasticDecisionRule();
        DeterministicDecisionRule ddr2 = DeterministicDecisionRule();

        for (const auto &type: *types2)
        {
            IloExpr probaSum(env); 
            distributionTotal = 0;
            ddr2.setProbability(type->toState(), actions2->toMultiDiscreteSpace()->getItem(0)->toAction());
            for (const auto &action: *actions2)
            {
                string varName = vn2.getVarNameStateAction(type->toState(), action->toAction());
                vars2.add(IloNumVar(env, 0, 1, ILOFLOAT, varName.c_str()));
                probaSum += vars2[inserted2];
                vn2.setNumber(varName, inserted2);
                inserted2++;
                // init strategy2
                double r = (double) rand() / (RAND_MAX);
                sdr2.setProbability(type->toState(), action->toAction(), r);
                distributionTotal += r;
            }
            model2.add(IloRange(env, 1, probaSum, 1));
            probaSum.end();
            
            //make initial distribution sum equal to 1
            for (const auto &action: *actions2){
                sdr2.setProbability(type->toState(), action->toAction(), sdr2.getProbability(type->toState(), action->toAction()) / distributionTotal);
            }
        }
        strategy2 = std::make_shared<StochasticDecisionRule>(sdr2);
        pureStrategy2 = std::make_shared<DeterministicDecisionRule>(ddr2);

        /* -------- ADD INITIAL CONSTRAINTS ----------- */
        for (const auto &t1: *types1){
            auto a1 = pureStrategy1->act(t1->toState());
            updateLP(t1->toState(), a1->toAction(),nullptr, nullptr, 1);
        } 
        for (const auto &t2: *types2){
            auto a2 = pureStrategy2->act(t2->toState());
            updateLP(nullptr, nullptr, t2->toState(), a2->toAction(), 0);
        } 
    }
    catch (IloException& e) {
        cerr << "[HS4BG->initLP()] Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }
}

double sdm::HS4BG::getActionProbability(std::shared_ptr<State> type, std::shared_ptr<Action> action, int agentId, bool fromInitialDistribution){
    if (fromInitialDistribution){
        if (agentId == 0){
            return strategy1->getProbability(type, action);
        }
        return strategy2->getProbability(type, action);
    }

    try
    {
        if (agentId == 0){
            auto varName = vn1.getVarNameStateAction(type, action);
            return cplex1.getValue(vars1[vn1.getNumber(varName)]);
        } else {
            auto varName = vn2.getVarNameStateAction(type, action);
            return cplex2.getValue(vars2[vn2.getNumber(varName)]);
        }    
    }
    catch (IloException& e) {
        cerr << "[HS4BG->getActionProbability(...)] Concert exception caught: " << e << endl;
    }
   catch (...) {
        cerr << "Unknown exception caught" << endl;
    }
    return 0;
}

std::shared_ptr<sdm::State> sdm::HS4BG::naiveHS(int agentId){
    auto agentTypes = game->getTypeSpace()->toMultiDiscreteSpace()->getSpace(agentId);
    double sumProba = 0;
    int i = 0;
    double r = (double) rand() / (RAND_MAX);
    for (const auto &t: *agentTypes){
        sumProba += game->getIndivTypeProba(t->toState(), agentId);
        if (r < sumProba) return agentTypes->toMultiDiscreteSpace()->getItem(i)->toState();
        i++;
    }
    return nullptr;
}

std::shared_ptr<sdm::Action> sdm::HS4BG::bestResponse(std::shared_ptr<State> type, int agentId, int step){

    std::shared_ptr<MultiDiscreteSpace> typeSpace = game->getTypeSpace()->toMultiDiscreteSpace();
    auto opTypes = typeSpace->getSpace(abs(agentId-1));

    std::shared_ptr<MultiDiscreteSpace> actionSpace = game->getActionSpace()->toMultiDiscreteSpace();
    auto opActions = actionSpace->getSpace(abs(agentId-1));
    auto agentActions = actionSpace->getSpace(agentId);

    float minPayoff = std::numeric_limits<float>::max();
    int bestResponseIndex;
    int i = 0;
    for (const auto &agentAction: *agentActions){
        float actionPayoff = 0;
        for (const auto &opType: *opTypes){
            Joint<std::shared_ptr<Item>> jointType(std::vector<std::shared_ptr<Item>>{opType, type});
            if (agentId == 0) jointType = Joint<std::shared_ptr<Item>>(std::vector<std::shared_ptr<Item>>{type, opType});
            auto jointState = typeSpace->getItemAddress(jointType)->toState();
            double jointTypeProba = game->getJointTypesProba(jointState);
            for (const auto &opAction: *opActions){
                Joint<std::shared_ptr<Item>> jAction(std::vector<std::shared_ptr<Item>>{opAction, agentAction});
                if (agentId == 0) jAction = Joint<std::shared_ptr<Item>>(std::vector<std::shared_ptr<Item>>{agentAction, opAction});
                auto jointAction = actionSpace->getItemAddress(jAction)->toAction();
                double opposingProbability = getActionProbability(opType->toState(), opAction->toAction(), abs(agentId-1), step == 0);
                actionPayoff += jointTypeProba*opposingProbability*(game->getPayoff(jointState, jointAction, abs(agentId-1)));
            }
        }
        if (actionPayoff < minPayoff){
            minPayoff = actionPayoff;
            bestResponseIndex = i;
        }
        i++;
    }
    return agentActions->toMultiDiscreteSpace()->getItem(bestResponseIndex)->toAction();
}


// TODO
// les LP doivent contenir au moins une contrainte pour chaque type du joueur opposé mais pas chaque action.
// C'est justement au fur et à mesure des itération qu'on rajoute des contraintes pour ces actions
// :param lpIndex: 0 update only LP1 | 1 update only LP2 | 2 update both LP
void sdm::HS4BG::updateLP(std::shared_ptr<State> type1, std::shared_ptr<Action> action1, std::shared_ptr<State> type2, std::shared_ptr<Action> action2, int lpIndex = 2){
    std::shared_ptr<MultiDiscreteSpace> typeSpace = game->getTypeSpace()->toMultiDiscreteSpace();
    std::shared_ptr<MultiDiscreteSpace> actionSpace = game->getActionSpace()->toMultiDiscreteSpace();
    
    auto types1 = typeSpace->getSpace(0);
    auto types2 = typeSpace->getSpace(1);
    auto actions1 = actionSpace->getSpace(0);
    auto actions2 = actionSpace->getSpace(1);
    
    // LP Agent 1
    if (lpIndex == 0 || lpIndex == 2){
        IloExpr actionConstraint(env);
        for (const auto &t1: *types1)
        {
            Joint<std::shared_ptr<Item>> jointType(std::vector<std::shared_ptr<Item>>{t1, type2});
            auto jointState = typeSpace->getItemAddress(jointType)->toState(); // cast à vérifier
            double jointTypeProba = game->getJointTypesProba(jointState);
            for (const auto &a1: *actions1)
            {
                Joint<std::shared_ptr<Item>> jAction(std::vector<std::shared_ptr<Item>>{a1, action2});
                auto jointAction = actionSpace->getItemAddress(jAction)->toAction();
                double coef = jointTypeProba * game->getPayoff(jointState, jointAction, 0);
                string varName = vn1.getVarNameStateAction(t1->toState(), a1->toAction());

                actionConstraint += coef * vars1[vn1.getNumber(varName)];
            };
        }
        int optiIndex = vn1.getNumber(vn1.getVarNameState(type2->toState()));
        actionConstraint -= vars1[optiIndex];
        model1.add(IloRange(env, 0, actionConstraint, IloInfinity));
        actionConstraint.end();
    }
    
    if (lpIndex == 1  || lpIndex == 2){
        IloExpr actionConstraint2(env);
        // LP Agent 2
        for (const auto &t2: *types2)
        {
            Joint<std::shared_ptr<Item>> jointType(std::vector<std::shared_ptr<Item>>{type1, t2});
            auto jointState = typeSpace->getItemAddress(jointType)->toState();
            double jointTypeProba = game->getJointTypesProba(jointState);
            for (const auto &a2: *actions2)
            {

                Joint<std::shared_ptr<Item>> jAction(std::vector<std::shared_ptr<Item>>{action1, a2});
                auto jointAction = actionSpace->getItemAddress(jAction)->toAction();

                double coef = jointTypeProba * game->getPayoff(jointState, jointAction, 1);
                string varName = vn2.getVarNameStateAction(t2->toState(), a2->toAction());

                actionConstraint2 += coef * vars2[vn2.getNumber(varName)];
            };
        }
        int optiIndex = vn2.getNumber(vn2.getVarNameState(type1->toState()));
        actionConstraint2 -= vars2[optiIndex];
        model2.add(IloRange(env, 0, actionConstraint2, IloInfinity));
        actionConstraint2.end();
    }
    

}

void sdm::HS4BG::updateStrategies(){
    cplex1 = IloCplex(model1);
    cplex1.setOut(env.getNullStream());
    if ( !cplex1.solve() ) {
        cout << "HS4BG : could not solve LP for Agent 1" << endl;
        throw(-1);
    }
    cplex2 = IloCplex(model2);
    cplex2.setOut(env.getNullStream());
    if ( !cplex2.solve() ) {
        cout << "HS4BG : could not solve LP for Agent 2" << endl;
        throw(-1);
    }
}

void sdm::HS4BG::solve(){
    float EPSILON = 0.1;
    float v1 = 1;
    float v2 = 0;
    int step = 0;
    while (abs(abs(v1) - abs(v2)) > EPSILON)
    {
        auto chosenType1 = naiveHS(0);
        auto chosenType2 = naiveHS(1);
        auto chosenAction1 = bestResponse(chosenType1, 0, step);
        auto chosenAction2 = bestResponse(chosenType2, 1, step);    

        updateLP(chosenType1, chosenAction1, chosenType2, chosenAction2);

        updateStrategies();

        v1 = cplex1.getObjValue();
        v2 = cplex2.getObjValue();

        std::cout << "[HS4BG] LP difference : " << abs(abs(v1) - abs(v2)) << std::endl;

        step++;
    }
    std::cout << "[HS4BG] solved in " << step << "steps" << std::endl;
}

void sdm::HS4BG::terminate(){
    env.end();
}

std::string sdm::HS4BG::getAlgorithmName(){ return "HS4BG";}

void sdm::HS4BG::test() {
    throw sdm::exception::NotImplementedException();
}

void sdm::HS4BG::save() {
    throw sdm::exception::NotImplementedException();
}

void sdm::HS4BG::saveSolution(){
    
    IloNumArray vals1(env);
    cplex1.getValues(vals1, vars1);
    IloNumArray vals2(env);
    cplex2.getValues(vals2, vars2);

    StochasticDecisionRule sdr1 = StochasticDecisionRule();
    int nTypes = game->getTypesNumbers()[0];
    int opposingTypes = game->getTypesNumbers()[1];
    int plActions = game->getGameDimensions()[0];
    for (int i = 0; i < nTypes; i ++)
    {
        for (int j = 0; j < plActions; j ++){
            sdr1.setProbability(std::make_shared<DiscreteState>(DiscreteState(i)), std::make_shared<DiscreteAction>(DiscreteAction(j)), vals1[opposingTypes + (i)*plActions + j]);
        }
    }
    strategy1 = std::make_shared<StochasticDecisionRule>(sdr1);

    StochasticDecisionRule sdr2 = StochasticDecisionRule();
    nTypes = opposingTypes;
    opposingTypes = game->getTypesNumbers()[0];
    plActions = game->getGameDimensions()[1];
    for (int i = 0; i < nTypes; i ++)
    {
        for (int j = 0; j < plActions; j ++){
            sdr2.setProbability(std::make_shared<DiscreteState>(DiscreteState(i)), std::make_shared<DiscreteAction>(DiscreteAction(j)), vals2[opposingTypes + (i)*plActions + j]);
        }
    }
    strategy2 = std::make_shared<StochasticDecisionRule>(sdr2);

}

std::vector<std::shared_ptr<sdm::StochasticDecisionRule>> sdm::HS4BG::getSolution(){
    return std::vector<std::shared_ptr<StochasticDecisionRule>>{strategy1, strategy2};
}
