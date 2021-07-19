
#include <sdm/utils/global_Test.hpp>
// #include <sdm/utils/value_function/initializer/initializers.hpp>

// #include <sdm/utils/value_function/point_set_value_function.hpp>
// #include <sdm/utils/value_function/hyperplan_value_function.hpp>

// #include <sdm/utils/value_function/backup/maxplan_backup.hpp>
// #include <sdm/utils/value_function/backup/tabular_backup.hpp>

// #include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
// #include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

// #include <sdm/parser/parser.hpp>
// #include <sdm/exception.hpp>
// #include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/algorithms.hpp>
#include <sdm/types.hpp>

int main(int argc, char **argv)
{
    std::vector<std::string> all_formalism={"decpomdp"};
    std::vector<std::string> all_problem={"recycling"};
    std::vector<int> all_horizon={20};
    std::vector<double> all_discount={1};
    std::vector<std::string> upper_bound_name = {"sawtooth"};
    std::vector<std::string> lower_bound_name={"maxplan"};
    std::vector<std::string> all_lower__init_name={"Min"};
    std::vector<std::string> all_upper_init_name= {"MdpHsvi"};

    int mean = 1;
    std::string filepath = "../data/world/dpomdp/";
    std::string save_path = "../run/Resultat/resultat";

    std::vector<std::string> all_sawtooth_current_type_of_resolution = {"IloIfThen"};
    std::vector<sdm::number> all_sawtooth_BigM = {1000};
    std::vector<std::string> all_sawtooth_type_of_linear_program = {"Full"};

    std::vector<int> all_truncation = {1};
    std::vector<int> all_freq_prunning_lower_bound = {-1};
    std::vector<sdm::TypeOfMaxPlanPrunning> all_type_of_maxplan_prunning = {sdm::TypeOfMaxPlanPrunning::PAIRWISE};
    std::vector<int> all_freq_prunning_upper_bound = {-1,1,5,10,50};
    std::vector<sdm::TypeOfSawtoothPrunning> all_type_of_sawtooth_prunning = {sdm::TypeOfSawtoothPrunning::GLOBAL};

    sdm::test(all_formalism,all_problem,all_horizon,all_discount,upper_bound_name,lower_bound_name,all_lower__init_name,all_upper_init_name,all_truncation,all_sawtooth_current_type_of_resolution,all_sawtooth_BigM,all_sawtooth_type_of_linear_program,all_type_of_maxplan_prunning,all_freq_prunning_lower_bound,all_type_of_sawtooth_prunning,all_freq_prunning_upper_bound,mean,filepath,save_path);

    return 0;
} // END main
