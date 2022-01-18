#include <iostream>
#include <limits>
#include <fstream>
#include <algorithm>
#include <sdm/world/two_players_bayesian_game.hpp>
#include <sdm/types.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>

sdm::TwoPlayersBayesianGame::TwoPlayersBayesianGame(){
    gameDimensions = std::vector<int>();
    typesNumbers = std::vector<int>();
}

sdm::number sdm::TwoPlayersBayesianGame::getNumAgents() const {
    return 2;
}

std::shared_ptr<sdm::Space> sdm::TwoPlayersBayesianGame::getTypeSpace() const{
    return types;
}

void sdm::TwoPlayersBayesianGame::setTypeNumbers(std::vector<std::string> strTypes){
    // update typesNumbers
    transform(strTypes.begin(), strTypes.end(), std::back_inserter(typesNumbers),
        [](const std::string& str) { return std::stoi(str); });

    // store types
    std::vector<std::shared_ptr<DiscreteState>> statesAgent1;
    for (int i = 0; i < typesNumbers[0]; i ++){
        statesAgent1.push_back(std::make_shared<DiscreteState>(i));
    }
    std::vector<std::shared_ptr<DiscreteState>> statesAgent2;
    for (int i = 0; i < typesNumbers[1]; i ++){
        statesAgent2.push_back(std::make_shared<DiscreteState>(i));
    }
    std::vector<std::shared_ptr<Space>> mds;
    mds.push_back(std::make_shared<DiscreteSpace>(statesAgent1));
    mds.push_back(std::make_shared<DiscreteSpace>(statesAgent2));

    types = std::make_shared<MultiDiscreteSpace>(mds);

}

void sdm::TwoPlayersBayesianGame::setGameDimensions(std::vector<std::string> strMatrixDimensions){
    //update game dimensions
    transform(strMatrixDimensions.begin(), strMatrixDimensions.end(), std::back_inserter(gameDimensions),
        [](const std::string& str) { return std::stoi(str); });

    //store actions
    std::vector<std::shared_ptr<DiscreteAction>> actionsAgent1;
    for (int i = 0; i < gameDimensions[0]; i ++){
        actionsAgent1.push_back(std::make_shared<DiscreteAction>(i));
    }
    std::vector<std::shared_ptr<DiscreteAction>> actionsAgent2;
    for (int i = 0; i < gameDimensions[1]; i ++){
        actionsAgent2.push_back(std::make_shared<DiscreteAction>(i));
    }
    std::vector<std::shared_ptr<Space>> mds;
    mds.push_back(std::make_shared<DiscreteSpace>(actionsAgent1));
    mds.push_back(std::make_shared<DiscreteSpace>(actionsAgent2));

    actions = std::make_shared<MultiDiscreteSpace>(mds);
}

std::shared_ptr<sdm::Space> sdm::TwoPlayersBayesianGame::getActionSpace() const{
    return actions;
}

void sdm::TwoPlayersBayesianGame::setPayoffs(std::vector<std::vector<std::string>> payoffElements) {
    int nbActionP1 = gameDimensions[0];
    int nbActionP2 = gameDimensions[1];
    int inserted = 0;
    int nfGameDone = 0;
    for (auto jointType : *types){
        for (auto jointAction: *actions) {
            for (int agentId = 0; agentId < 2; agentId ++){
                payoffs[jointType->toState()][jointAction->toAction()][agentId] = std::stof(payoffElements[((int) inserted/nbActionP2) + nbActionP1*(agentId + nfGameDone)][inserted % nbActionP2]);
            }
            inserted++;
        }
        nfGameDone++;
    }
}

void sdm::TwoPlayersBayesianGame::setJointTypeProbabilities(std::vector<std::vector<std::string>> probabilities) {
    int inserted = 0;
    int p2Types = typesNumbers[1];
    for (auto jointType : *types){
        jTypeProbabilities[jointType->toState()] = std::stof(probabilities[(int) inserted / p2Types][inserted % p2Types]);
        inserted++;
    }
}

double sdm::TwoPlayersBayesianGame::getJointTypesProba(std::shared_ptr<State> joint_type) {
    try
    {
        return jTypeProbabilities[joint_type];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}

double sdm::TwoPlayersBayesianGame::getIndivTypeProba(std::shared_ptr<State> type, int agentId){
    try {
        double proba = 0;
        auto opTypes = types->toMultiDiscreteSpace()->getSpace(abs(agentId-1));
        for (const auto &opType: *opTypes)
        {
            Joint<std::shared_ptr<Item>> jointType(std::vector<std::shared_ptr<Item>>{opType, type});
            if (agentId == 0) jointType = Joint<std::shared_ptr<Item>>(std::vector<std::shared_ptr<Item>>{type, opType});
            auto jointState = types->toMultiDiscreteSpace()->getItemAddress(jointType)->toState();
            proba += getJointTypesProba(jointState);        
        }
        return proba;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
};


float sdm::TwoPlayersBayesianGame::getPayoff(std::shared_ptr<State> types, std::shared_ptr<Action> actions, int idAgent){
    try {
        return payoffs[types][actions][idAgent];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return - std::numeric_limits<float>::infinity();
}

std::vector<int> sdm::TwoPlayersBayesianGame::getGameDimensions(){
    return gameDimensions;
}

std::vector<int> sdm::TwoPlayersBayesianGame::getTypesNumbers(){
    return typesNumbers;
}
