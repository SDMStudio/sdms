#include <iostream>

#include <sdm/config.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/core/state/jhistory_tree.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename = (argc > 1) ? argv[1] : config::PROBLEM_PATH + "dpomdp/tiger.dpomdp";
    std::shared_ptr<MPOMDP> mpomdp = parser::parse_file(filename);

    std::cout << "----- Usage : class Joint ( sdm/core/state/jhistory_tree.hpp ) ---------" << std::endl
              << std::endl;

    // Instanciate a joint history tree of max depth 3
    std::shared_ptr<JointHistoryTree> jhistory = std::make_shared<JointHistoryTree>(mpomdp->getNumAgents(), 3);

    // Get basic elements of joint histories
    std::cout << "\n--- 1) Basic access" << std::endl;

    std::cout << "\n#> Number of agents = " << jhistory->getNumAgents() << std::endl;
    std::cout << "#> Horizon = " << jhistory->getHorizon() << std::endl; // equivalent to jhistory->getDepth()
    std::cout << "#> MaxDepth = " << jhistory->getMaxDepth() << std::endl;
    std::cout << "#> Initial Joint history : " << *jhistory << std::endl;

    // How to expand a joint history
    std::cout << "\n--- 2) Instanciate and expand a joint history" << std::endl;

    std::shared_ptr<JointHistoryInterface> joint_history = jhistory;
    // List of joint observation for the example
    for (const auto &joint_obs : *mpomdp->getObservationSpace(0))
    {
        std::cout << "\n#> Expand with observation " << *joint_obs << std::endl;
        joint_history = joint_history->expand(joint_obs->toObservation())->toJointHistory();
        std::cout << "#> Expanded joint history --> " << *joint_history << std::endl;
    }

    std::cout << "\n#> Get Last Observation " << *joint_history->getLastObservation() << std::endl;

    // How to access individual histories and expand them
    std::cout << "\n--- 3) Access individual histories" << std::endl;

    std::cout << "\n#> List of pointer on individual histories = " << joint_history->getIndividualHistories() << std::endl;

    for (number agent_id = 0; agent_id < mpomdp->getNumAgents(); ++agent_id)
    {
        std::cout << "#> IndividualHistory(" << agent_id << ") = " << *jhistory->getIndividualHistory(agent_id) << std::endl; // equivalent to jhistory->get(agent_id)
    }

    return 0;
}
