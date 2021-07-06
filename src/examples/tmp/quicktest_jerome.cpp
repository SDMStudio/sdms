// #include <iostream>

// #include <memory>
// #include <sdm/exception.hpp>

// // #include <sdm/algorithms/hsvi.hpp>

// #include <sdm/core/action/action.hpp>
// #include <sdm/core/action/base_action.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/base_state.hpp>
// #include <sdm/core/state/serialized_state.hpp>
// #include <sdm/core/base_observation.hpp>

// #include <sdm/core/space/discrete_space.hpp>
// #include <sdm/core/space/multi_discrete_space.hpp>
// #include <sdm/core/distribution.hpp>
// #include <sdm/core/reward/tabular_reward.hpp>
// #include <sdm/core/dynamics/tabular_state_dynamics.hpp>

// #include <sdm/world/mdp.hpp>
// #include <sdm/world/mmdp.hpp>
// #include <sdm/world/solvable_by_mdp.hpp>
// #include <sdm/world/serialized_mmdp.hpp>
// #include <sdm/world/mpomdp.hpp>
// #include <sdm/world/pomdp.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/occupancy_mdp.hpp>

// #include <sdm/parser/parser.hpp>

// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/utils/value_function/point_set_value_function.hpp>
// #include <sdm/utils/value_function/hyperplan_value_function.hpp>

// #include <sdm/utils/value_function/backup/maxplan_backup.hpp>
// #include <sdm/utils/value_function/backup/tabular_backup.hpp>

// #include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
// #include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
// #include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

// #include <sdm/utils/value_function/initializer/initializer.hpp>
// #include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
// #include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

// #include <sdm/world/serialized_mpomdp.hpp>

// #include <sdm/core/state/history_tree.hpp>
// // #include <sdm/core/state/jhistory_tree.hpp>

// #include <sdm/utils/value_function/initializer/initializers.hpp>

#include <sdm/algorithms.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    std::string path = "../data/world/dpomdp/mabc.dpomdp";

    number horizon = 5;
    double discount = 1;
    double error = 0.01;
    int truncation = 2;

    std::string formalism ="decpomdp";
    std::string upper_bound ="";
    std::string lower_bound ="";
    std::string ub_init ="PomdpHsvi";
    std::string lb_unit ="Min";

    auto algorithm = sdm::algo::make("hsvi",path,formalism,upper_bound,lower_bound,ub_init,lb_unit,discount,error,horizon,1000,truncation);
    algorithm->do_initialize();

    algorithm->do_solve(); 

    return 0;
} // END main
