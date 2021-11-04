
#include <sdm/algorithms/planning/perseus.hpp>

namespace sdm
{
    Perseus::Perseus(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, number num_sample_states, number size_by_step, double error, double time_max, std::string name)
        : ValueIteration(world, value_function, error, time_max, name), num_sample_states(num_sample_states), size_by_step(size_by_step)
    {
    }

    void Perseus::initTrial()
    {
    }

    void Perseus::initialize()
    {
        initLogger();
        value_function->initialize();
    }

    std::shared_ptr<State> Perseus::selectOneState(number t)
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

    std::shared_ptr<Space> Perseus::selectStates(number t)
    {
        if (isInstanceOf<MDPInterface>(this->getWorld()))
        {
            return std::dynamic_pointer_cast<MDPInterface>(getWorld())->getStateSpace(t);
        }
        else
        {
            std::vector<std::shared_ptr<State>> list_states;
            for (int i = 0; i < num_sample_states; i++)
            {
                list_states.push_back(this->selectOneState(t));
            }
            return std::make_shared<DiscreteSpace>(list_states);
        }
    }

    bool Perseus::stop()
    {
        return (getValueFunction()->getSize() > (size_by_step * getWorld()->getHorizon()));
    }

    void Perseus::updateValue(const std::shared_ptr<State> &state, number t)
    {
        getValueFunction()->updateValueAt(state, t);
    }

    std::string Perseus::getAlgorithmName()
    {
        return "Perseus";
    }

}