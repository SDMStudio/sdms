#include <iostream>
#include <sdm/types.hpp>

#include <sdm/utils/struct/graph.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_state_vector.hpp>
#include <sdm/core/state/belief_state_graph.hpp>

#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/core/states.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
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
        clock_t t_begin = clock();

        // using TState = number;
        // using TObservation = number;

        // using TActionDescriptor = number;
        // using TStateDescriptor = HistoryTree_p<TObservation>;

        // using TActionPrescriptor = JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;

        // using TStatePrescriptor = SerializedOccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>;
        // // using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

        // // Construct OccupancyMDP using parser
        // std::cout << "#> Parsing file \"" << filename << "\"\n";
        // auto omdp_world = std::make_shared<SerializedOccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(filename);

        // // We will show how to expand an initial occupancy state and generate next ones using compression
        // int depth = 0, limit = 10;
        // // std::cout << "#> Print depth \"" << depth << "\"\n";
        // auto ostate = omdp_world->getInitialState();

        // // std::cout << "#> Print occupancy state \n"
        // //           << ostate << "\n";

        // auto oaction = omdp_world->getActionSpaceAt(ostate)->sample();
        // // std::cout << "#> Print joint decision rule \n"
        // //           << oaction << "\n";

        // do
        // {
        //     depth++;
        //     std::cout << "#> Print depth \"" << depth << "\"\n";

        //     // Compute the next compressed occupancy state
        //     ostate = omdp_world->nextState(ostate, oaction);
        //     // std::cout << "#> Print compressed occupancy state \n"
        //     //           << ostate << "\n";
        //     // std::cout << "#> Print one step left occupancy state \n"
        //     //           << *ostate.getOneStepUncompressedOccupancy() << "\n";
        //     // std::cout << "#> Print fully uncompressed occupancy state \n"
        //     //           << *ostate.getFullyUncompressedOccupancy() << "\n";

        //     // Sample a decision rule
        //     oaction = omdp_world->getActionSpaceAt(ostate)->sample();
        //     // std::cout << "#> Print joint decision rule \n"
        //     //           << oaction << "\n";
        // } while (depth < limit);
        // std::cout << "Time : " << ((float)(clock() - t_begin) / CLOCKS_PER_SEC) << std::endl;
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}
