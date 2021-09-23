
#include <iostream>
#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/world/gym/RobotBin.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    int sizeX = (argc > 1) ? std::stoi(argv[1]) : 3;
    int sizeY = (argc > 2) ? std::stoi(argv[2]) : sizeX;

    std::shared_ptr<GymInterface> env = std::make_shared<gym::RobotBin>(sizeX, sizeY);
    // TEST GRAPH
    std::cout << "\n--------- Usage : class GymInterface ( sdm/world/gym_interface.hpp ) ---------\n\n";

    std::shared_ptr<Observation> observation = env->reset(), next_observation;
    std::vector<double> reward;
    bool is_done;

    auto action_space = env->getActionSpaceAt(observation, 0);
    std::cout << config::LOG_SDMS << "action_space=" << action_space->str() << std::endl;
    std::cout << config::LOG_SDMS << "T=0" << std::endl;
    std::cout << "\t\tobservation=" << observation->str() << std::endl;

    for (number t = 0; t < 10; t++)
    {
        auto action = action_space->sample()->toAction();
        std::cout << "\t\taction=" << action->str() << std::endl;
        std::tie(next_observation, reward, is_done) = env->step(action);
        std::cout << config::LOG_SDMS << "T=" << t+1 << std::endl;
        std::cout << "\t\tobservation=" << next_observation->str() << "\treward=" << reward << "\tis_done=" << is_done << std::endl;
    }
}