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
    payoffMatrixes = std::vector<std::vector<double>>();
    jointTypeProbabilities = std::vector<std::vector<double>>();
}

sdm::number sdm::TwoPlayersBayesianGame::getNumAgents() const {
    return 2;
}

void sdm::TwoPlayersBayesianGame::setTypeNumbers(std::vector<std::string> strTypes){
    transform(strTypes.begin(), strTypes.end(), std::back_inserter(typesNumbers),
        [](const std::string& str) { return std::stoi(str); });
}

void sdm::TwoPlayersBayesianGame::setGameDimensions(std::vector<std::string> strMatrixDimensions){
    transform(strMatrixDimensions.begin(), strMatrixDimensions.end(), std::back_inserter(gameDimensions),
        [](const std::string& str) { return std::stoi(str); });
}

void sdm::TwoPlayersBayesianGame::addPayoffLine(std::vector<std::string> strPayoffs){
    payoffMatrixes.push_back({});
    transform(strPayoffs.begin(), strPayoffs.end(), std::back_inserter(payoffMatrixes[payoffMatrixes.size()-1]),
    [](const std::string& str) { return std::stof(str);});
}

void sdm::TwoPlayersBayesianGame::addJointTypeProbabilities(std::vector<std::string> strProbabilities){
    jointTypeProbabilities.push_back({});
    transform(strProbabilities.begin(), strProbabilities.end(), std::back_inserter(jointTypeProbabilities[jointTypeProbabilities.size()-1]),
    [](const std::string& str) { return std::stof(str);});
}

float sdm::TwoPlayersBayesianGame::getJointTypesProba(std::vector<std::shared_ptr<State>> types){
    try
    {
        DiscreteState& type1 = dynamic_cast<DiscreteState&>(*types[0]); 
        DiscreteState& type2 = dynamic_cast<DiscreteState&>(*types[1]); 
        return jointTypeProbabilities[type1.getState()][type2.getState()];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return -1;
}

float sdm::TwoPlayersBayesianGame::getPayoff(std::vector<std::shared_ptr<State>> types, std::vector<std::shared_ptr<Action>> actions, int idAgent){
    auto type1 = dynamic_cast<DiscreteState&>(*types[0]); 
    DiscreteState& type2 = dynamic_cast<DiscreteState&>(*types[1]); 
    DiscreteAction& action1 = dynamic_cast<DiscreteAction&>(*actions[0]);
    DiscreteAction& action2 = dynamic_cast<DiscreteAction&>(*actions[1]);
    int playerOffset = 0;
    if (idAgent == 1) playerOffset = gameDimensions[0];
    try {
        return payoffMatrixes[type1.getState()*typesNumbers[1]*gameDimensions[0]*2 + type2.getState()*gameDimensions[0]*2 + action1.getAction() + playerOffset][action2.getAction()];
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
