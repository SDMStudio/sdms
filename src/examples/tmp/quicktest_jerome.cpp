
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

int main(int argc, char **argv)
{
    std::vector<std::string> all_formalism={"decpomdp"};
    std::vector<std::string> all_problem={"tiger"};
    std::vector<int> all_horizon={3};
    std::vector<double> all_discount={1};
    std::vector<std::string> upper_bound_name = {""};
    std::vector<std::string> lower_bound_name={"maxplan"};
    std::vector<std::string> all_lower__init_name={"Min"};
    std::vector<std::string> all_upper_init_name= {"MdpHsvi"};

    int mean = 1;
    std::string filepath = "../data/world/dpomdp/";
    std::string save_path = "../run/Resultat/resultat";

    std::vector<std::string> all_sawtooth_current_type_of_resolution = {"IloIfThen"};
    std::vector<sdm::number> all_sawtooth_BigM = {1000};
    std::vector<std::string> all_sawtooth_type_of_linear_program = {"Relaxed"};

    std::vector<int> all_truncation = {3};
    std::vector<int> all_freq_prunning_lower_bound = {-1,5,10,50};
    std::vector<int> all_freq_prunning_upper_bound = {-1};

    sdm::test(all_formalism,all_problem,all_horizon,all_discount,upper_bound_name,lower_bound_name,all_lower__init_name,all_upper_init_name,all_truncation,all_sawtooth_current_type_of_resolution,all_sawtooth_BigM,all_sawtooth_type_of_linear_program,all_freq_prunning_lower_bound,all_freq_prunning_upper_bound,mean,filepath,save_path);


    // std::string filename = "../data/world/dpomdp/mabc.dpomdp";
    // int horizon = 50;
    // int discount = 1;
    // double error = 0.001;
    // int trials = 1000;
	// int truncation = 1;

    // int freq_prunning = -1;
    

    // auto problem = sdm::parser::parse_file(filename);
    // problem->setHorizon(horizon);
    // problem->setDiscount(discount);

    // std::shared_ptr<sdm::SolvableByHSVI> oMDP = std::make_shared<sdm::OccupancyMDP>(problem, (truncation > 0) ? truncation : horizon);

    // auto tabular_backup = std::make_shared<sdm::TabularBackup>(oMDP);
    // auto maxplan_backup = std::make_shared<sdm::MaxPlanBackup>(oMDP);

    // auto action_maxplan = std::make_shared<sdm::ActionVFMaxplan>(oMDP);
    // auto action_tabular = std::make_shared<sdm::ActionVFTabulaire>(oMDP);

    // auto init_lb = std::make_shared<sdm::MinInitializer>(oMDP);
    // auto init_ub = std::make_shared<sdm::MaxInitializer>(oMDP);

    // // Instanciate bounds
    // std::shared_ptr<sdm::ValueFunction> lower_bound = std::make_shared<sdm::HyperplanValueFunction>(horizon,init_lb,maxplan_backup,action_maxplan,freq_prunning);
    // std::shared_ptr<sdm::ValueFunction> upper_bound = std::make_shared<sdm::TabularValueFunction>(horizon,init_ub,tabular_backup, action_tabular);

    // auto algo = std::make_shared<sdm::HSVI>(oMDP, lower_bound, upper_bound, problem->getHorizon(), error, trials);

    // algo->do_initialize();
    // algo->do_solve();

    return 0;
} // END main
