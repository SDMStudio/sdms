#include <iostream>
#include <sdm/core/joint.hpp>
#include <sdm/core/state/base_state.hpp>

using namespace sdm;

int main(int, char **)
{
    std::cout << "----- Usage : class Joint ( sdm/core/joint.hpp ) ---------" << std::endl
              << std::endl;

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

    std::shared_ptr<State> state_0 = std::make_shared<DiscreteState>(0), state_1 = std::make_shared<DiscreteState>(1),
                           state_2 = std::make_shared<DiscreteState>(2), state_3 = std::make_shared<DiscreteState>(3);

    Joint joint_state({state_0, state_1, state_2});

    std::cout << "\n\n#> Joint state : " << joint_state << std::endl;
    std::cout << "#> NumAgents : " << joint_state.getNumAgents() << std::endl;
    for (number agent_id = 0; agent_id < joint_state.getNumAgents(); ++agent_id)
    {
        std::cout << "#> State(" << agent_id << ") = " << joint_state.get(agent_id)->str() << std::endl;
    }
    std::cout << "#> Add \"" << *state_1 << "\" in the list of joint state." << std::endl;
    joint_state.push_back(state_1);
    std::cout << "#> New joint observation : " << joint_state << std::endl;
    std::cout << "#> NumAgents : " << joint_state.getNumAgents() << std::endl;

    return 0;
} // END main