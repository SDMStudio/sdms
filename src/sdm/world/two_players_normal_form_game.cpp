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
    payoffMatrixes = std::vector<std::vector<double>>();
    jointTypeProbabilities = std::vector<std::vector<double>>{{1},{1}};
}

sdm::number sdm::TwoPlayersNormalFormGame::getNumAgents() const {
    return 2;
}

void sdm::TwoPlayersNormalFormGame::setGameDimensions(std::vector<std::string> strMatrixDimensions){
    transform(strMatrixDimensions.begin(), strMatrixDimensions.end(), std::back_inserter(gameDimensions),
        [](const std::string& str) { return std::stoi(str); });
}

void sdm::TwoPlayersNormalFormGame::addPayoffLine(std::vector<std::string> strPayoffs){
    payoffMatrixes.push_back({});
    transform(strPayoffs.begin(), strPayoffs.end(), std::back_inserter(payoffMatrixes[payoffMatrixes.size()-1]),
    [](const std::string& str) { return std::stof(str);});
}

float sdm::TwoPlayersNormalFormGame::getJointTypesProba(std::vector<std::shared_ptr<State>> types){
    DiscreteState& type1 = dynamic_cast<DiscreteState&>(*types[0]); 
    DiscreteState& type2 = dynamic_cast<DiscreteState&>(*types[1]); 
    if (type1.getState() != 0 || type2.getState() != 0) return 0;
    try
    {
        return jointTypeProbabilities[type1.getState()][type2.getState()];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return -1;
}

float sdm::TwoPlayersNormalFormGame::getPayoff(std::vector<std::shared_ptr<State>> types, std::vector<std::shared_ptr<Action>> actions, int whichPlayer){
    DiscreteState& type1 = dynamic_cast<DiscreteState&>(*types[0]); 
    DiscreteState& type2 = dynamic_cast<DiscreteState&>(*types[1]); 
    if (type1.getState() != 0 || type2.getState() != 0) return - std::numeric_limits<float>::infinity();;
    DiscreteAction& action1 = dynamic_cast<DiscreteAction&>(*actions[0]);
    DiscreteAction& action2 = dynamic_cast<DiscreteAction&>(*actions[1]);    int playerOffset = 0;
    if (whichPlayer == 1) playerOffset = gameDimensions[0];
    try {
        return payoffMatrixes[type1.getState()*typesNumbers[1]*gameDimensions[0]*2 + type2.getState()*gameDimensions[0]*2 + action1.getAction() + playerOffset][action2.getAction()];
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
