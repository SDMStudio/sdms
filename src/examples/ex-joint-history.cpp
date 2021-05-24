#include <iostream>
#include <sdm/core/joint.hpp>
#include <sdm/core/state/history.hpp>
#include <sdm/core/state/jhistory_tree.hpp>

using namespace sdm;

int main(int, char **)
{
    std::cout << "----- Usage : class Joint ( sdm/core/state/jhistory_tree.hpp ) ---------" << std::endl
              << std::endl;

    using TObservation = number;

    number num_agents = 2, max_depth = 3;

    JointHistoryTree_p<TObservation> jhistory(new JointHistoryTree<TObservation>(num_agents, max_depth));

    // Get basic elements of joint histories
    std::cout << "\n--- 1) Basic access" << std::endl;

    std::cout << "\n#> Number of agents = " << jhistory->getNumAgents() << std::endl;
    std::cout << "#> Horizon = " << jhistory->getHorizon() << std::endl; // equivalent to jhistory->getDepth()
    std::cout << "#> MaxDepth = " << jhistory->getMaxDepth() << std::endl;
    std::cout << "#> Initial Joint history : " << *jhistory << std::endl;

    // How to expand a joint history
    std::cout << "\n--- 2) Instanciate and expand a joint history" << std::endl;

    // List of joint observation for the example
    std::vector<Joint<TObservation>> list_joint_obs = {{1, 0}, {0, 0}, {0, 0}, {2, 1},{36,20}};
    for (const auto &joint_obs : list_joint_obs)
    {
        std::cout << "\n#> Expand with observation " << joint_obs << std::endl;
        jhistory = jhistory->expand(joint_obs);
        std::cout << "#> Expanded joint history --> " << *jhistory << std::endl;
        std::cout<<"\n Get Parent of Joint History "<<*jhistory->getParent()<<std::endl;
    }

    std::cout<<"\n get Last Observation "<<jhistory->getData()<<std::endl;

    std::cout<<"\n Get Parent of Joint History "<<*jhistory->getParent()<<std::endl;

    // How to access individual histories and expand them
    std::cout << "\n--- 3) Access individual histories" << std::endl;

    std::cout << "\n#> List of pointer on individual histories = " << jhistory->getIndividualHistories() << std::endl;

    for (number agent_id = 0; agent_id < jhistory->getNumAgents(); ++agent_id)
    {
        std::cout << "#> IndividualHistory(" << agent_id << ") = " << *jhistory->getIndividualHistory(agent_id) << std::endl; // equivalent to jhistory->get(agent_id)
    }

    return 0;
}
