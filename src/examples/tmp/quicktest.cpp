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

#include <sdm/utils/global_Test.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    try
    {
        // Grid3x3corners

        std::vector<std::string> all_formalism={"decpomdp"};
        std::vector<std::string> all_problem={"mabc","tiger","recycling","GridSmall","alignment_2x4","Mars"};
        std::vector<int> all_horizon={10};
        std::vector<double> all_discount={1};
        std::vector<std::string> upper_bound_name = {"sawtooth_lp"};
        std::vector<std::string> lower_bound_name={"maxplan_lp"};
        std::vector<std::string> all_lower__init_name={"BlindInitializer"};
        std::vector<std::string> all_upper_init_name= {"MdpValueIterationInitializer"};
        std::vector<int> all_truncation = {2};
        std::vector<std::string> all_sawtooth_current_type_of_resolution = {"BigM"};
        std::vector<number> all_sawtooth_BigM = {1000};
        int mean = 1;
        std::string filepath = "../data/world/dpomdp/";
        std::string save_path = "../run/Resultat/resultat.csv";

        test(all_formalism,all_problem,all_horizon,all_discount,upper_bound_name,lower_bound_name,all_lower__init_name, all_upper_init_name,
            all_truncation,all_sawtooth_current_type_of_resolution, all_sawtooth_BigM ,mean,filepath ,save_path); 
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}
