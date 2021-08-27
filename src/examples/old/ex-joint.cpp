#include <iostream>
#include <sdm/core/joint.hpp>

using namespace sdm;

int main(int, char **)
{
    std::cout << "----- Usage : class Joint ( sdm/core/joint.hpp ) ---------" << std::endl << std::endl;

    Joint joint_observation({1, 2, 3});

    std::cout << "#> Joint observation : " << joint_observation << std::endl;
    std::cout << "#> NumAgents : " << joint_observation.getNumAgents() << std::endl;
    for (number agent_id = 0; agent_id < joint_observation.getNumAgents(); ++agent_id)
    {
        std::cout << "#> Observation(" << agent_id << ") = " << joint_observation.get(agent_id) << std::endl;
    }
    std::cout << "#> Add \"5\" in the list of joint observation." << std::endl;
    joint_observation.push_back(5);
    std::cout << "#> New joint observation : " << joint_observation << std::endl;
    std::cout << "#> NumAgents : " << joint_observation.getNumAgents() << std::endl;

    return 0;
} // END main