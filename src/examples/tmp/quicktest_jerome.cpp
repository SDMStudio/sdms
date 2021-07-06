#include <sdm/algorithms.hpp>

#include <sdm/utils/global_Test.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    std::vector<std::string> all_formalism={"decpomdp"};
    std::vector<std::string> all_problem={"tiger"};
    std::vector<int> all_horizon={5};
    std::vector<double> all_discount={1};
    std::vector<std::string> upper_bound_name = {"","sawtooth"};
    std::vector<std::string> lower_bound_name={"","maxplan","maxplan_lp"};
    std::vector<std::string> all_lower__init_name={"Min"};
    std::vector<std::string> all_upper_init_name= {"Max","Mdp","Pomdp"};
    int mean = 2;
    std::string filepath = "../data/world/dpomdp/";
    std::string save_path = "../run/Resultat/resultat.csv";

    test(all_formalism,all_problem,all_horizon,all_discount,upper_bound_name,lower_bound_name,all_lower__init_name,all_upper_init_name,mean,filepath,save_path);

    // std::string path = "../data/world/dpomdp/mabc.dpomdp";

    // number horizon = 5;
    // double discount = 1;
    // double error = 0.01;
    // int truncation = 2;

    // std::string formalism ="decpomdp";
    // std::string upper_bound ="sawtooth";
    // std::string lower_bound ="maxplan_lp";
    // std::string ub_init ="PomdpHsvi";
    // std::string lb_unit ="Min";

    // auto algorithm = sdm::algo::make("hsvi",path,formalism,upper_bound,lower_bound,ub_init,lb_unit,discount,error,horizon,1000,truncation);
    // algorithm->do_initialize();

    // algorithm->do_solve(); 

    return 0;
} // END main
