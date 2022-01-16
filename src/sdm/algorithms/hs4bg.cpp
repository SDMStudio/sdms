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

        int inserted2 = 0;
        for (const auto &t1: *types1)
        {
            string varName = vn2.getVarNameState(t1->toState());
            vars2.add(IloNumVar(env, -IloInfinity, IloInfinity, ILOFLOAT, varName.c_str()));
            obj += vars2[inserted2];
            vn2.setNumber(varName, inserted2);
            inserted2++;
        }
        // add objective to model
        model2.add(IloMaximize(env,obj));
        obj.end();

        /* --------- ADD SDR OPTI VARS AND CORRESPONDING CONSTRAINTS FOR BOTH LP ------------ */
        StochasticDecisionRule sdr1 = StochasticDecisionRule();
        double distributionTotal;
        for (const auto &type: *types1)
        {
            IloExpr probaSum(env); 
            distributionTotal = 0;
            for (const auto &action: *actions1)
            {
                string varName = vn1.getVarNameStateAction(type->toState(), action->toAction());
                vars1.add(IloNumVar(env, 0, 1, ILOFLOAT, varName.c_str()));
                probaSum += vars1[inserted1];
                vn1.setNumber(varName, inserted1);
                inserted1++;
                // init strategy1
                double r = (double) rand() / (RAND_MAX);
                sdr1.setProbability(type, action, r);
                distributionTotal += r;
            }
            model1.add(IloRange(env, 1, probaSum, 1));
            probaSum.end();

            //make initial distribution sum equal to 1
            for (const auto &action: *actions1){
                sdr1.setProbability(type, action, sdr1.getProbability(type, action) / distributionTotal);
            }
        }
        strategy1 = std::make_shared<StochasticDecisionRule>(sdr1);

        StochasticDecisionRule sdr2 = StochasticDecisionRule();
        for (const auto &type: *types2)
        {
            IloExpr probaSum(env); 
            distributionTotal = 0;
            for (const auto &action: *actions2)
            {
                string varName = vn2.getVarNameStateAction(type->toState(), action->toAction());
                vars2.add(IloNumVar(env, 0, 1, ILOFLOAT, varName.c_str()));
                probaSum += vars2[inserted2];
                vn2.setNumber(varName, inserted2);
                inserted2++;
                // init strategy2
                double r = (double) rand() / (RAND_MAX);
                sdr2.setProbability(type, action, r);
                distributionTotal += r;
            }
            model2.add(IloRange(env, 1, probaSum, 1));
            probaSum.end();
            
            //make initial distribution sum equal to 1
            for (const auto &action: *actions2){
                sdr2.setProbability(type, action, sdr2.getProbability(type, action) / distributionTotal);
            }
        }
        strategy2 = std::make_shared<StochasticDecisionRule>(sdr2);

    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
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
            IloCplex c(model1);
            return c.getValue(vals1[vn1.getNumber(varName)]);
        } else {
            auto varName = vn2.getVarNameStateAction(type, action);
            IloCplex c(model2);
            return c.getValue(vals2[vn2.getNumber(varName)]);
        }    
    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
   catch (...) {
        cerr << "Unknown exception caught" << endl;
    }
    
    
}

std::shared_ptr<State> sdm::HS4BG::naiveHS(int agentId){
    auto agentTypes = game->getTypeSpace()->toMultiDiscreteSpace()->getSpace(agentId);
    double maxProba = 0;
    std::shared_ptr<State> mostLikelyState;
    for (const auto &t: *agentTypes){
        double typeProba = game->getIndivTypeProba(t, agentId);
        if (typeProba > maxProba){
            maxProba = typeProba;
            mostLikelyState = t;
        }
    }
    return mostLikelyState;
}

std::shared_ptr<Action> sdm::HS4BG::bestResponse(std::shared_ptr<State> type, int agentId, int step){

    auto opTypes = game->getTypeSpace()->toMultiDiscreteSpace()->getSpace(abs(agentId-1));
    std::shared_ptr<MultiDiscreteSpace> actionSpace = game->getActionSpace()->toMultiDiscreteSpace();
    auto opActions = actionSpace->getSpace(abs(agentId-1));
    auto agentActions = actionSpace->getSpace(agentId);

    float minPayoff = std::numeric_limits<float>::max();
    std::shared_ptr<Action> bestResponse;
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
                double opposingProbability = getActionProbability(opType, opAction, abs(agentId-1), step == 0);
                actionPayoff += jointTypeProba*opposingProbability*(game->getPayoff(jointState, jointAction, abs(agentId-1)));
            }
        }
        if (actionPayoff < minPayoff){
            minPayoff = actionPayoff;
            bestResponse = agentAction;
        }
    }
    return agentActions;
}

void sdm::HS4BG::updateLP(std::shared_ptr<State> type1, std::shared_ptr<Action> action1, std::shared_ptr<State> type2, std::shared_ptr<Action> action2){
    std::shared_ptr<MultiDiscreteSpace> typeSpace = game->getTypeSpace()->toMultiDiscreteSpace();
    std::shared_ptr<MultiDiscreteSpace> actionSpace = game->getActionSpace()->toMultiDiscreteSpace();
    
    auto types1 = typeSpace->getSpace(0);
    auto types2 = typeSpace->getSpace(1);
    auto actions1 = actionSpace->getSpace(0);
    auto actions2 = actionSpace->getSpace(1);
    
    IloExpr actionConstraint(env);  
    // LP Agent 1
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
            string varName = vn1.getVarNameStateAction(jointState->toState(), jointAction->toAction());

            actionConstraint += coef * vars1[vn1.getNumber(varName)];
        };
    }
    int optiIndex = vn1.getNumber(vn1.getVarNameState(type2->toState()));
    actionConstraint -= vars1[optiIndex];
    model1.add(IloRange(env, 0, actionConstraint, IloInfinity));
    actionConstraint.end();

    // LP Agent 2
    for (const auto &t2: *types2)
    {
        Joint<std::shared_ptr<Item>> jointType(std::vector<std::shared_ptr<Item>>{type1, t2});
        auto jointState = typeSpace->getItemAddress(jointType)->toState(); // cast à vérifier
        double jointTypeProba = game->getJointTypesProba(jointState);
        for (const auto &a2: *actions2)
        {

            Joint<std::shared_ptr<Item>> jAction(std::vector<std::shared_ptr<Item>>{action1, a2});
            auto jointAction = actionSpace->getItemAddress(jAction)->toAction();

            double coef = jointTypeProba * game->getPayoff(jointState, jointAction, 1);
            string varName = vn2.getVarNameStateAction(jointState->toState(), jointAction->toAction());

            actionConstraint += coef * vars2[vn2.getNumber(varName)];
        };
    }
    optiIndex = vn2.getNumber(vn2.getVarNameState(type1->toState()));
    actionConstraint -= vars2[optiIndex];
    model2.add(IloRange(env, 0, actionConstraint, IloInfinity));
    actionConstraint.end();

}

void sdm::HS4BG::updateStrategies(){
    IloCplex c1(model1);
    if ( !c1.solve() ) {
        cout << "HS4BG : could not solve LP for Agent 1" << endl;
        throw(-1);
    }
    IloCplex c2(model2);
    if ( !c2.solve() ) {
        cout << "HS4BG : could not solve LP for Agent 2" << endl;
        throw(-1);
    }
}

void sdm::HS4BG::solve(){
    float EPSILON = 0.1;
    float vSup = 1;
    float vInf = 0;
    int step = 0;
    while (vSup - vInf > EPSILON)
    {
        auto chosenType1 = naiveHS(0);
        auto chosenType2 = naiveHS(1);
        auto chosenAction1 = bestResponse(chosenType1, 0, step);
        auto chosenAction2 = bestResponse(chosenType2, 1), step;        

        updateLP(chosenType1, chosenAction1, chosenType2, chosenAction2);

        updateStrategies();

        IloCplex c1(model1);
        IloCplex c2(model2);
        
        vSup = c1.getValue(vals1[0]);
        vInf = c2.getValues(vals2[0]);
        step++;
    }
    
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
    
    StochasticDecisionRule sdr1 = StochasticDecisionRule();
    int nTypes = game->getTypesNumbers()[0];
    int opposingTypes = game->getTypesNumbers()[1];
    int plActions = game->getGameDimensions()[0];
    for (int i = 0; i < nTypes; i ++)
    {
        for (int j = 0; j < plActions; j ++){
            sdr1.setProbability(std::make_shared<DiscreteState>(DiscreteState(i)), std::make_shared<DiscreteAction>(DiscreteAction(j)), vals1[opposingTypes + (i)*plActions + j + 1]);
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
            sdr2.setProbability(std::make_shared<DiscreteState>(DiscreteState(i)), std::make_shared<DiscreteAction>(DiscreteAction(j)), vals2[opposingTypes + (i)*plActions + j + 1]);
        }
    }
    strategy2 = std::make_shared<StochasticDecisionRule>(sdr2);

}

std::vector<std::shared_ptr<StochasticDecisionRule>> sdm::HS4BG::getSolution(){
    return std::vector<std::shared_ptr<StochasticDecisionRule>>{strategy1, strategy2};
}