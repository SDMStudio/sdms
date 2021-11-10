#include <iostream>
#include <limits>
#include <fstream>
#include <algorithm>
#include "two_players_bayesian_game.hpp"

float sdm::TwoPlayersBayesianGame::getJointTypeProba(int type1, int type2){
    try
    {
        return jointTypeProbabilities[type1][type2];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return -1;
}

float sdm::TwoPlayersBayesianGame::getPayoff(int type1, int type2, int action1, int action2, int whichPlayer){
    int playerOffset = 0;
    if (whichPlayer == 1) playerOffset = matrixDimensions[0];
    try {
        return payoffMatrixes[type1*typesNumbers[1]*matrixDimensions[0]*2 + type2*matrixDimensions[0]*2 + action1 + playerOffset][action2];
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return std::numeric_limits<float>::infinity();
}

std::vector<int> sdm::TwoPlayersBayesianGame::getMatrixDimensions(){
    return matrixDimensions;
}

std::vector<int> sdm::TwoPlayersBayesianGame::getTypesNumbers(){
    return typesNumbers;
}
