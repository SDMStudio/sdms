#include <iostream>
#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/utils/struct/graph.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // TEST GRAPH
    std::cout << "\n--------- Usage : class Graph ( sdm/utils/struct/graph.hpp ) ---------\n\n";

    Graph<int, double> graph;

    graph.addNode(1);
    graph.addNode(2);
    graph.addNode(3);

    graph.addSuccessor(1, 0.3, 2);
    graph.addSuccessor(1, 0.7, 3);
    graph.addSuccessor(3, 1.0, 2);

    std::cout << config::LOG_SDMS << "NumNodes=" << graph.getNumNodes() << std::endl; // OUTPUT : 3
    std::cout << config::LOG_SDMS << "NumSucc(1)=" << graph.getNode(1)->getNumSuccessors() << std::endl; // OUTPUT : 2
    std::cout << config::LOG_SDMS << "Succ(1, 0.7)=" << graph.getSuccessor(1, 0.7)->getData() << std::endl; // OUTPUT : 3
    std::cout << config::LOG_SDMS << "Succ(Succ(1, 0.7), 1.0)=" << graph.getSuccessor(1, 0.7)->getSuccessor(1.0)->getData() << std::endl; // OUTPUT : 2

}
