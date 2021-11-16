
#include <sdm/algorithms/planning/pbvi.hpp>

namespace sdm
{
    PBVI::PBVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, number num_sample_states, double error, double time_max, std::string name)
        : ValueIteration(world, value_function, error, time_max, name), num_sample_states(num_sample_states)
    {
    }

    void PBVI::initialize()
    {
        ValueIteration::initialize();
        initStateSpace();
    }

    void PBVI::initStateSpace()
    {
        for (number t = 0; t < getWorld()->getHorizon(); t++)
        {

            std::vector<std::shared_ptr<State>> list_states;
            for (int i = 0; i < num_sample_states; i++)
            {
                list_states.push_back(selectOneState(t));
            }
            sampled_state_space.push_back(std::make_shared<DiscreteSpace>(list_states));
        }
    }

    std::shared_ptr<State> PBVI::selectOneState(number t)
    {
        std::shared_ptr<State> current_state = getWorld()->getInitialState();
        std::shared_ptr<Action> current_action;
        std::shared_ptr<Observation> current_observation;
        for (number h = 0; h < t; h++)
        {
            current_action = getWorld()->getActionSpaceAt(current_state, t)->sample()->toAction(); //getWorld()->getRandomAction(current_state, t);
            current_observation = getWorld()->getObservationSpaceAt(current_state, current_action, t)->sample()->toObservation();
            current_state = getWorld()->getNextStateAndProba(current_state, current_action, current_observation, t).first;
        }
        return current_state;
    }

    std::shared_ptr<Space> PBVI::selectStates(number t)
    {
        return sampled_state_space[t];
    }

    std::string PBVI::getAlgorithmName()
    {
        return "PBVI";
    }

}