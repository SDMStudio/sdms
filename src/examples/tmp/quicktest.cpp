#include <sdm/utils/global_Test.hpp>

#include <sdm/core/state/serialized_belief_state.hpp>
#include <sdm/world/serialized_belief_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // try
    // {
    //     number horizon = 2;

    //     using TState = BeliefState;
    //     using TAction = number;

    //     auto sbmdp = std::make_shared<BeliefMDP<TState, TAction>>("../data/world/dpomdp/tiger.dpomdp");

    //     auto algo = sdm::algo::makeHSVI<TState,TAction>(sbmdp,"","","MaxInitializer","MinInitializer",1,0,2,1000,"");

    //     algo->do_initialize();
    //     algo->do_solve();

    //     //auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();

    //     //auto lower_bound = std::make_shared<sdm::MaxPlanValueFunction<TState, TAction>>(beliefMDP, horizon, lb_init);
    // }
    // catch (sdm::exception::Exception &e)
    // {
    //     std::cout << "!!! Exception: " << e.what() << std::endl;
    // }


    std::string filePath("../data/world/dpomdp/");

    std::vector<std::string> all_problem = {"mabc","tiger","recycling"};

    std::vector<std::string> all_formalism = {"extensive-decpomdp","decpomdp"};

    std::vector<int> all_horizon = {3};

    std::vector<double> all_discount = {1};

    std::vector<std::string> all_lower_bound = {"MinInitializer"};

    std::vector<std::string> all_upper_bound = {"MdpHsviInitializer"};
    
    sdm::test(all_formalism, all_problem,all_horizon,all_discount,{"sawtooth"},{""},all_lower_bound,all_upper_bound,1,filePath);

}