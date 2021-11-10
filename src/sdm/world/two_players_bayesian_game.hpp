#include <vector>

#ifndef TWOPLAYERSBAYESIANGAME

namespace sdm
{
    class TwoPlayersBayesianGame
    {
        public:

        float getJointTypeProba(int type1, int type2);

        float getPayoff(int type1, int type2, int action1, int action2, int whichPlayer);
            
        std::vector<int> getMatrixDimensions();

        std::vector<int> getTypesNumbers();

        std::vector<int> matrixDimensions;
        std::vector<int> typesNumbers;
        std::vector<std::vector<float>> payoffMatrixes;
        std::vector<std::vector<float>> jointTypeProbabilities;


    };
};

#define TWOPLAYERSBAYESIANGAME
#endif