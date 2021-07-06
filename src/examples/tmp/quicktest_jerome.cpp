#include <sdm/algorithms.hpp>

#include <sdm/utils/global_Test.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    std::vector<std::string> all_formalism={"decpomdp"};
    std::vector<std::string> all_problem={"tiger","mabc","recycling"};
    std::vector<int> all_horizon={3,5};
    std::vector<double> all_discount={1};
    std::vector<std::string> upper_bound_name = {"","sawtooth"};
    std::vector<std::string> lower_bound_name={"","maxplan","maxplan_lp"};
    std::vector<std::string> all_lower__init_name={"Min"};
    std::vector<std::string> all_upper_init_name= {"PomdpHsvi"};
    int mean = 2;
    std::string filepath = "../data/world/dpomdp/";
    std::string save_path = "../run/Resultat/NewSDMS/resultat.csv";

    test(all_formalism,all_problem,all_horizon,all_discount,upper_bound_name,lower_bound_name,all_lower__init_name,all_upper_init_name,mean,filepath,save_path);

    return 0;
} // END main
