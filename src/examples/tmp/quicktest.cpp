#include <sdm/utils/global_Test.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // try
    // {
    //     using TState = OccupancyState<number, JointHistoryTree_p<number>>;
    //     using TAction = Joint<DeterministicDecisionRule<HistoryTree_p<number>, number>>;

    //     auto somdp = std::make_shared<OccupancyMDP<TState, TAction>>(filePath+filename+".dpomdp", length_history);

    //     number horizon = 2;
    //     auto beliefMDP = std::make_shared<BeliefMDP<TState, TAction>>(filename);
    //     auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();

    //     auto lower_bound = std::make_shared<sdm::MaxPlanValueFunction<TState, TAction>>(beliefMDP, horizon, lb_init);
    // }
    // catch (sdm::exception::Exception &e)
    // {
    //     std::cout << "!!! Exception: " << e.what() << std::endl;
    // }
    std::string filePath("../data/world/dpomdp/");

    std::vector<std::string> all_problem = {"mabc","tiger"}; //,"tiger","recycling"};

    std::vector<std::string> all_formalism = {"mdp"};

    std::vector<int> all_horizon = {2};

    std::vector<double> all_discount = {1};

    std::vector<std::string> all_lower_bound = {"MinInitializer"};

    std::vector<std::string> all_upper_bound = {"MaxInitializer"};
    
    sdm::test(all_formalism, all_problem,all_horizon,all_discount,{""},{""},all_lower_bound,all_upper_bound,1,filePath);

}