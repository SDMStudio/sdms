#include <iostream>
#include <sdm/types.hpp>
#include <sdm/utils/struct/graph.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_state_graph.hpp>
#include <sdm/world/discrete_decpomdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // TEST GRAPH
    std::cout << "\n--------- Usage : class Graph ( sdm/utils/struct/graph.hpp ) ---------\n\n";

    auto graph = std::make_shared<Graph<number, number>>(std::make_shared<number>(7));

    std::cout << "#> Graph : " << *graph << std::endl;

    graph->addSuccessor(1, std::make_shared<number>(2));
    graph->addSuccessor(2, std::make_shared<number>(4));
    graph->addSuccessor(3, std::make_shared<number>(9));

    std::cout << "#> Graph : " << *graph << std::endl;
    std::cout << "#> Succ Graph : " << *graph->getSuccessor(1) << std::endl;
    std::cout << "#> Succ Graph : " << *graph->getSuccessor(3) << std::endl;
    graph = graph->getSuccessor(1);

    // TEST BeliefStateGraph
    std::cout << "\n--------- Usage : class BeliefStateGraph ( sdm/core/state/belief_state_graph.hpp ) ---------\n\n";

    std::string filename;

    if (argc > 1)
    {
        filename = argv[1];
    }
    else
    {
        std::cerr << "Error: Require 1 input file." << std::endl;
        return 1;
    }

    try
    {
        DiscreteDecPOMDP world(filename);

        number num_states = world.getStateSpace()->getNumItems();

        // Build belief
        auto ptr_belief = std::make_shared<Vector>(num_states);
        for (number i = 0; i < num_states; i++)
        {
            (*ptr_belief)[i] = world.getStartDistrib().probabilities()[i];
        }
        auto belief = std::make_shared<BeliefStateGraph<Vector, number, number>>(ptr_belief);
        belief->setDynamics(world.getObsDynamics()->getDynamics());

        std::cout << "#> Belief (addr) : " << belief << std::endl;
        std::cout << "#> Belief : " << *belief << std::endl;
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}
