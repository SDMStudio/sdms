#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/algorithms/q_learning.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;
    number horizon, length_history;

    if (argc > 2)
    {
        filename = argv[1];
        horizon = std::atoi(argv[2]);
        length_history = horizon;

        if (argc > 3)
        {
            length_history = std::atoi(argv[3]);
        }
    }
    else
    {
        std::cerr << "Error:  arg[1] must be an input file, arg[2] must be the horizon, arg[3] is optional (the length of history)." << std::endl;
        return 1;
    }

    auto environment = std::make_shared<DiscreteMDP>(filename);

    // Instanciate bounds
    std::shared_ptr<sdm::QValueFunction<number, number>> qvalue(new sdm::MappedQValueFunction<number, number>(horizon));
    std::shared_ptr<sdm::QValueFunction<number, number>> target_qvalue(new sdm::MappedQValueFunction<number, number>(horizon));

    QLearning<number, number, number> algo(environment, qvalue, target_qvalue, params...);

    algo->do_solve();

    return 0;
}