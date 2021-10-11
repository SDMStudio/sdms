
#include <sdm/utils/global_Test.hpp>
// #include <sdm/utils/value_function/initializer/initializers.hpp>

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

#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>

// #include <sdm/algorithms.hpp>
#include <sdm/types.hpp>

#include <sdm/algorithms/backward_induction.hpp>

#include <sdm/utils/value_function/tabular_qvalue_function_conditioning.hpp>
#include <sdm/algorithms/alpha_star.hpp>


using namespace sdm;

int main(int argc, char **argv)
{

    std::string problem_path = "../data/world/dpomdp/Mars.dpomdp";
    int horizon = 4;
    double discount = 1;
    int memory =1;

    auto problem = sdm::parser::parse_file(problem_path);

    problem->setHorizon(horizon);
    problem->setDiscount(discount);

    auto serial_mpomdp = std::make_shared<SerialMPOMDP>(problem);
    std::shared_ptr<SolvableByHSVI> mdp = std::make_shared<SerialOccupancyMDP>(serial_mpomdp,memory);

    auto algo_backward = std::make_shared<BackwardInduction>(mdp,mdp->getUnderlyingProblem()->getHorizon());
    auto algo_hsvi = sdm::algo::makeHSVI(mdp,"Tabular","Tabular","PomdpHsvi","Min",discount,0,mdp->getUnderlyingProblem()->getHorizon());
    auto algo_alpha_star = std::make_shared<AlphaStar>(mdp,"A*");

    algo_hsvi->initialize();
    algo_alpha_star->initialize();
    algo_backward->initialize();

    std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();

    algo_hsvi->solve();

    std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    double TOTAL_TIME_HSVI = std::chrono::duration_cast<std::chrono::duration<double>>(t_end-t_begin).count();

    t_begin = std::chrono::high_resolution_clock::now();

    // algo_alpha_star->solve();

    t_end = std::chrono::high_resolution_clock::now();
    double TOTAL_TIME_ALPHASTAR = std::chrono::duration_cast<std::chrono::duration<double>>(t_end-t_begin).count();

    t_begin = std::chrono::high_resolution_clock::now();

    // algo_backward->solve();

    t_end = std::chrono::high_resolution_clock::now();

    double TOTAL_TIME_BACKWARD = std::chrono::duration_cast<std::chrono::duration<double>>(t_end-t_begin).count();

    // std::cout<<"Bound Hsvi "<<algo_hsvi->getUpperBound()->str()<<std::endl;
    // algo_hsvi->do_test();
    // std::cout<<"Bound Backward"<<algo_alpha_star->getBound()->str()<<std::endl;

    std::cout<<"Total Time HSVI "<<TOTAL_TIME_HSVI<<std::endl;
    std::cout<<"Total Time Backward "<<TOTAL_TIME_BACKWARD<<std::endl;
    std::cout<<"Total Time Alpha "<<TOTAL_TIME_ALPHASTAR<<std::endl;




    return 0;
} // END main
