
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
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp_serial.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_serial.hpp>

#include <sdm/utils/value_function/action_vf/action_sawtooth_wcsp.hpp>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>

#include <sdm/algorithms.hpp>
#include <sdm/types.hpp>

#include <sdm/algorithms/backward_induction.hpp>

using namespace sdm;

int main(int argc, char **argv)
{

    std::string problem_path = "../data/world/dpomdp/recycling.dpomdp";
    int horizon = 5;
    double discount = 1;
    int memory =1;

    auto problem = sdm::parser::parse_file(problem_path);

    problem->setHorizon(horizon);
    problem->setDiscount(discount);

    auto serialized_mpomdp = std::make_shared<SerializedMPOMDP>(problem);
    std::shared_ptr<SolvableByHSVI> mdp = std::make_shared<SerialOccupancyMDP>(serialized_mpomdp,memory);

    auto algo_backward = std::make_shared<BackwardInduction>(mdp,serialized_mpomdp->getHorizon());
    auto algo_hsvi = sdm::algo::makeHSVI(mdp,"Tabular","Tabular","Max","Min",discount,0,serialized_mpomdp->getHorizon());

    std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();

    algo_hsvi->do_initialize();
    algo_hsvi->do_solve();

    std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    double TOTAL_TIME_HSVI = std::chrono::duration_cast<std::chrono::duration<double>>(t_end-t_begin).count();

    t_begin = std::chrono::high_resolution_clock::now();

    algo_backward->do_initialize();
    algo_backward->do_solve();

    t_end = std::chrono::high_resolution_clock::now();
    double TOTAL_TIME_BACKWARD = std::chrono::duration_cast<std::chrono::duration<double>>(t_end-t_begin).count();

    // std::cout<<"Bound Hsvi "<<algo_hsvi->getUpperBound()->str()<<std::endl;
    // std::cout<<"Bound Backward"<<algo_backward->getBound()->str()<<std::endl;

    std::cout<<"Total Time HSVI "<<TOTAL_TIME_HSVI<<std::endl;
    std::cout<<"Total Time Backward "<<TOTAL_TIME_BACKWARD<<std::endl;


    return 0;
} // END main
