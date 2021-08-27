#include <iostream>
#include <sdm/types.hpp>
#include <sdm/utils/struct/graph.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_state_vector.hpp>
#include <sdm/core/state/belief_state_graph.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/utils/struct/recursive_map.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // TEST GRAPH
    std::cout << "\n--------- Usage : class Graph ( sdm/utils/struct/graph.hpp ) ---------\n\n";

    auto graph = std::make_shared<Graph<number, number>>(7);

    std::cout << "#> Graph : " << *graph << std::endl;

    graph->addSuccessor(1, 2);
    graph->addSuccessor(2, 4);
    graph->addSuccessor(3, 9);

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

        BeliefStateVector belief = world.getStartDistrib().probabilities();

        auto belief_graph = std::make_shared<BeliefStateGraph<BeliefStateVector, number, number>>(belief, world.getObsDynamics()->getDynamics());
        std::cout << "#> Belief (addr) : " << belief_graph << std::endl;
        belief_graph->initialize();

        std::cout << "#> Belief : " << belief_graph << " - " << *belief_graph << std::endl;

        belief_graph = belief_graph->expand(8, 3);
        std::cout << "#> Belief : " << belief_graph << " - " << *belief_graph << std::endl;
        belief_graph = belief_graph->expand(8, 0);
        std::cout << "#> Belief : " << belief_graph << " - " << *belief_graph << std::endl;
        belief_graph = belief_graph->expand(8, 0);
        std::cout << "#> Belief : " << belief_graph << " - " << *belief_graph << std::endl;
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}
