#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/utils/value_function/initializers.hpp>

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

    std::map<int, double> m = {{1, 3.5},
                               {4, 10}};

    std::shared_ptr<SolvableByHSVI<number, number>> mdp = std::make_shared<DiscreteMDP>(filename);
    // std::map<std::string, std::shared_ptr<sdm::Initializer<number, number>> (*)()> m2 = {
    //     {"MaxInitializer", &sdm::createInstance<number, number, sdm::MaxInitializer>},
    //     {"MinInitializer", &sdm::createInstance<number, number, sdm::MinInitializer>},
    // };

    // auto init = sdm::InitializerFactory<number, number>::make("MaxInitializer");
    std::cout << "Available Init" << std::endl;
    for (auto &v : sdm::InitializerFactory<number, number>::available())
    {
        std::cout << v << std::endl;
    }

    // sdm::InitializerFactory<number, number>::addToRegistry<sdm::BlindInitializer>("BlindInitializer");

    std::cout << "Available Init" << std::endl;

    std::cout << sdm::InitializerFactory<number, number>::available() << std::endl;

    // auto init2 = sdm::makeInitializer<number, number>("BlindInitializer");

    // using TState = SerializedOccupancyState<number, JointHistoryTree_p<number>>;
    // using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

    // auto upb = mdp->getUnderlyingProblem();

    // std::cout << *upb->getStateSpace() << std::endl;
    // std::cout << *upb->getActionSpace() << std::endl;
    // std::cout << *upb << std::endl;

    return 0;
}