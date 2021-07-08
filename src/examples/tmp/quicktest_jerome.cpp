
#include <sdm/utils/global_Test.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    std::vector<std::string> all_formalism={"decpomdp"};
    std::vector<std::string> all_problem={"mabc"};
    std::vector<int> all_horizon={5};
    std::vector<double> all_discount={1};
    std::vector<std::string> upper_bound_name = {"sawtooth"};
    std::vector<std::string> lower_bound_name={"maxplan"};
    std::vector<std::string> all_lower__init_name={"Min"};
    std::vector<std::string> all_upper_init_name= {"PomdpHsvi"};
    int mean = 2;
    std::string filepath = "../data/world/dpomdp/";
    std::string save_path = "../run/Resultat/NewSDMS/resultat.csv";

    std::vector<int> all_truncation = {1};
    std::vector<std::string> all_sawtooth_current_type_of_resolution = {"IloIfThen"};
    std::vector<number> all_sawtooth_BigM = {1000};

    test(all_formalism,all_problem,all_horizon,all_discount,upper_bound_name,lower_bound_name,all_lower__init_name,all_upper_init_name,all_truncation,all_sawtooth_current_type_of_resolution,all_sawtooth_BigM,mean,filepath,save_path);

    return 0;
} // END main
