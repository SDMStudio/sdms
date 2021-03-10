#include <iostream>
#include <cassert>
#include <stdio.h>  /* printf, scanf, puts, NULL */
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */

#include <tuple>
#include <typeinfo>
#include <sdm/common.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/world/discrete_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;

    if (argc > 1)
    {
        filename = argv[1];
        std::cout << "#> Parsing NDPOMDP file \"" << filename << "\"\n";
    }

    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    srand(time(NULL));

    // DiscreteMDP mdp(filename);
    auto mdp = std::make_shared<DiscretePOMDP>(filename);

    mdp->setPlanningHorizon(5);

    mdp->setupDynamicsGenerator();

    for (number ep = 0; ep < 10; ep++)
    {

        double reward;
        bool is_done = false;
        number obs;
        mdp->reset();
        // number obs = mdp->reset();
        // std::cout << "Observation initial : " << obs << std::endl;

        while (!is_done)
        {
            number random_action = rand() % mdp->getActionSpace()->getNumItems();
            std::cout << "Execute Action : " << random_action << std::endl;
            std::tie(obs, reward, is_done) = mdp->step(random_action);
            std::cout << "( " << obs << ", " << reward << ", " << is_done << " )" << std::endl;
        }
    }

    // NDPOMDP ndpomdp(filename);

    // std::cout << "--------------------------------" << std::endl;

    // std::cout << ndpomdp.getStateSpace() << std::endl;
    // std::cout << ndpomdp.getActionSpace() << std::endl;
    // std::cout << ndpomdp.getObsSpace() << std::endl;

    return 0;
}