#include <iostream>
#include <limits>
#include <fstream>
#include <algorithm>
#include <sdm/world/two_players_normal_form_game.hpp>

sdm::TwoPlayersNormalFormGame::TwoPlayersNormalFormGame(){
    gameDimensions = std::vector<int>();
    typesNumbers = std::vector<int>{1,1};
    payoffMatrixes = std::vector<std::vector<float>>();
    jointTypeProbabilities = std::vector<std::vector<float>>{{1},{1}};
}

int sdm::TwoPlayersNormalFormGame::getNombreAgents() {
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

float sdm::TwoPlayersNormalFormGame::getJointTypesProba(std::vector<int> types){
    if (types[0] != 0 || types[1] != 0) return 0;
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

float sdm::TwoPlayersNormalFormGame::getPayoff(std::vector<int> types, std::vector<int> actions, int whichPlayer){
    if (types[0] != 0 || types[1] != 0) return -1;
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

std::vector<int> sdm::TwoPlayersNormalFormGame::getGameDimensions(){
    return gameDimensions;
}

std::vector<int> sdm::TwoPlayersNormalFormGame::getTypesNumbers(){
    return typesNumbers;
}
