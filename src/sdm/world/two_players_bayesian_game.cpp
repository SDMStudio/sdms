#include <iostream>
#include <limits>
#include <fstream>
#include <algorithm>
#include <sdm/world/two_players_bayesian_game.hpp>

sdm::TwoPlayersBayesianGame::TwoPlayersBayesianGame(){
    gameDimensions = std::vector<int>();
    typesNumbers = std::vector<int>();
    payoffMatrixes = std::vector<std::vector<float>>();
    jointTypeProbabilities = std::vector<std::vector<float>>();
}

int sdm::TwoPlayersBayesianGame::getNombreAgents() {
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

float sdm::TwoPlayersBayesianGame::getJointTypesProba(std::vector<int> types){
    try
    {
        return jointTypeProbabilities[types[0]][types[1]];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return -1;
}

float sdm::TwoPlayersBayesianGame::getPayoff(std::vector<int> types, std::vector<int> actions, int whichPlayer){
    int type1(types[0]); int type2(types[1]);
    int action1(actions[0]); int action2(actions[1]);
    int playerOffset = 0;
    if (whichPlayer == 1) playerOffset = gameDimensions[0];
    try {
        return payoffMatrixes[type1*typesNumbers[1]*gameDimensions[0]*2 + type2*gameDimensions[0]*2 + action1 + playerOffset][action2];
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
