
#include <sdm/utils/global_Test.hpp>
#include <sdm/utils/value_function/initializer/initializers.hpp>

#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_wcsp.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp_relaxed.hpp>

#include <sdm/utils/value_function/action_vf/action_sawtooth_wcsp.hpp>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/types.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // std::vector<std::string> all_formalism={"decpomdp"};
    // std::vector<std::string> all_problem={"boxPushingUAI07"};
    // std::vector<int> all_horizon={5};
    // std::vector<double> all_discount={1};
    // std::vector<std::string> upper_bound_name = {"sawtooth_lp"};
    // std::vector<std::string> lower_bound_name={"maxplan_lp","maxplan_wcsp"};
    // std::vector<std::string> all_lower__init_name={"Min"};
    // std::vector<std::string> all_upper_init_name= {"PomdpHsvi"};

    // int mean = 1;
    // std::string filepath = "../data/world/dpomdp/";
    // std::string save_path = "../run/Resultat/resultat";

    // std::vector<std::string> all_sawtooth_current_type_of_resolution = {"IloIfThen"};
    // std::vector<sdm::number> all_sawtooth_BigM = {1000};
    // std::vector<std::string> all_sawtooth_type_of_linear_program = {"Full"};

    // std::vector<int> all_truncation = {2};
    // std::vector<int> all_freq_prunning_lower_bound = {-1};
    // std::vector<sdm::TypeOfMaxPlanPrunning> all_type_of_maxplan_prunning = {sdm::TypeOfMaxPlanPrunning::PAIRWISE};
    // std::vector<int> all_freq_prunning_upper_bound = {-1};
    // std::vector<sdm::TypeOfSawtoothPrunning> all_type_of_sawtooth_prunning = {sdm::TypeOfSawtoothPrunning::GLOBAL};

    // sdm::test(all_formalism,all_problem,all_horizon,all_discount,upper_bound_name,lower_bound_name,all_lower__init_name,all_upper_init_name,all_truncation,all_sawtooth_current_type_of_resolution,all_sawtooth_BigM,all_sawtooth_type_of_linear_program,all_type_of_maxplan_prunning,all_freq_prunning_lower_bound,all_type_of_sawtooth_prunning,all_freq_prunning_upper_bound,mean,filepath,save_path);

    std::string filename = "../data/world/dpomdp/tiger.dpomdp";
    int horizon = 3;
    int discount = 1;
    double error = 0.01;
    int trials = 1000;
	int truncation = 3;

    TypeOfResolution type_of_resolution = TypeOfResolution::IloIfThenResolution;

	auto ValueBigM = 100;

    auto problem = sdm::parser::parse_file(filename);
    problem->setHorizon(horizon);
    problem->setDiscount(discount);

    std::shared_ptr<SolvableByHSVI> oMDP = std::make_shared<OccupancyMDP>(problem, (truncation > 0) ? truncation : horizon, true,true,true);

    auto tabular_backup = std::make_shared<TabularBackup>(oMDP);
    auto maxplan_backup = std::make_shared<MaxPlanBackup>(oMDP);

    auto action_maxplan_wcsp = std::make_shared<ActionVFMaxplanWCSP>(oMDP);
    auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(oMDP);

    auto action_maxplan = std::make_shared<ActionVFMaxplan>(oMDP);

    auto action_tabular = std::make_shared<ActionVFTabulaire>(oMDP);
    auto action_sawtooth_lp =  std::make_shared<ActionVFSawtoothLP>(oMDP, type_of_resolution,ValueBigM);
    auto action_sawtooth_lp_relaxed =  std::make_shared<ActionVFSawtoothLPRelaxed>(oMDP, type_of_resolution);

    auto action_sawtooth_wcsp=  std::make_shared<ActionVFSawtoothWCSP>(oMDP);

    auto init_lb = std::make_shared<MinInitializer>(oMDP);
    auto init_ub = std::make_shared<MDPInitializer>(oMDP, "Pomdp Init",0);

    // Instanciate bounds
    std::shared_ptr<sdm::ValueFunction> lower_bound = std::make_shared<HyperplanValueFunction>(horizon,init_lb,maxplan_backup,action_maxplan_wcsp);
    std::shared_ptr<sdm::ValueFunction> upper_bound = std::make_shared<PointSetValueFunction>(horizon,init_ub,tabular_backup, action_sawtooth_lp);

    auto algo = std::make_shared<HSVI>(oMDP, lower_bound, upper_bound, problem->getHorizon(), error, trials);

    algo->do_initialize();
    algo->do_solve();

    // algo->do_test();

    return 0;
} // END main
