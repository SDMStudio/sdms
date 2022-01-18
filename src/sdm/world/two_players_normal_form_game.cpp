#include <iostream>
#include <limits>
#include <fstream>
#include <algorithm>
#include <sdm/world/two_players_normal_form_game.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>

sdm::TwoPlayersNormalFormGame::TwoPlayersNormalFormGame(){
    gameDimensions = std::vector<int>();
    typesNumbers = std::vector<int>{1,1};
        
    // build types
    std::vector<std::shared_ptr<DiscreteState>> statesAgent1;
    statesAgent1.push_back(std::make_shared<DiscreteState>(0));
    std::vector<std::shared_ptr<DiscreteState>> statesAgent2;
    statesAgent2.push_back(std::make_shared<DiscreteState>(0));

    std::vector<std::shared_ptr<Space>> mds;
    mds.push_back(std::make_shared<DiscreteSpace>(DiscreteSpace(statesAgent1)));
    mds.push_back(std::make_shared<DiscreteSpace>(DiscreteSpace(statesAgent2)));

    types = std::make_shared<MultiDiscreteSpace>(mds);

    for (auto jointType : *types){
        jTypeProbabilities[jointType->toState()] = 1;
    }
}

sdm::number sdm::TwoPlayersNormalFormGame::getNumAgents() const {
    return 2;
}

std::shared_ptr<sdm::Space> sdm::TwoPlayersNormalFormGame::getTypeSpace() const {
    return types;
}

void sdm::TwoPlayersNormalFormGame::setGameDimensions(std::vector<std::string> strMatrixDimensions){
    transform(strMatrixDimensions.begin(), strMatrixDimensions.end(), std::back_inserter(gameDimensions),
        [](const std::string& str) { return std::stoi(str); });
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

std::shared_ptr<sdm::Space> sdm::TwoPlayersNormalFormGame::getActionSpace() const{
    return actions;
}

void sdm::TwoPlayersNormalFormGame::setPayoffs(std::vector<std::vector<std::string>> payoffElements) {
    int nbActionP1 = gameDimensions[0];
    int nbActionP2 = gameDimensions[1];
    int inserted = 0;
    for (auto jointType : *types){
        for (auto jointAction: *actions) {
            for (int agentId = 0; agentId < 2; agentId ++){
                payoffs[jointType->toState()][jointAction->toAction()][agentId] = std::stof(payoffElements[((int) inserted/nbActionP2) + nbActionP1*agentId][inserted % nbActionP2]);
            }
            inserted++;
        }
    }
}

double sdm::TwoPlayersNormalFormGame::getJointTypesProba(std::shared_ptr<State> joint_type) {
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

double sdm::TwoPlayersNormalFormGame::getIndivTypeProba(std::shared_ptr<State> type, int agentId){
    return 1;
}


float sdm::TwoPlayersNormalFormGame::getPayoff(std::shared_ptr<State> types, std::shared_ptr<Action> actions, int idAgent){
    try {
        return payoffs[types][actions][idAgent];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return - std::numeric_limits<float>::infinity();
}

std::vector<int> sdm::TwoPlayersNormalFormGame::getGameDimensions(){
    return gameDimensions;
}

std::vector<int> sdm::TwoPlayersNormalFormGame::getTypesNumbers(){
    return typesNumbers;
}
