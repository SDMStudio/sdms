
#include <sdm/algorithms/planning/pbvi.hpp>

namespace sdm
{
    PBVI::PBVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, number num_sample_states, double error, double time_max, std::string name, std::string type_sampling)
        : ValueIteration(world, value_function, error, time_max, name), num_sample_states(num_sample_states), type_sampling(type_sampling)
    {
    }

    void PBVI::initialize()
    {
        ValueIteration::initialize();
        if (type_sampling == "1")
            initStateSpace2();
        else
            initStateSpace();
    }

    std::shared_ptr<State> PBVI::sampleNextState(const std::shared_ptr<State> &state, number t)
    {

        std::shared_ptr<Action> sampled_action = std::dynamic_pointer_cast<GymInterface>(getWorld())->getRandomAction(state, t);

        std::shared_ptr<State> candidate_state = nullptr;

        // Get a random number between 0 and 1
        double epsilon = std::rand() / (double(RAND_MAX)), cumul = 0., proba;

        // Go over all observations of the lower-level agent
        auto obs_space = getWorld()->getObservationSpaceAt(state, sampled_action, t);
        for (auto obs_n : *obs_space)
        {
            std::tie(candidate_state, proba) = getWorld()->getNextStateAndProba(state, sampled_action, obs_n->toObservation(), t);

            cumul += proba;
            if (epsilon < cumul)
            {
                return candidate_state;
            }
        }

        return candidate_state;
    }

    void PBVI::initStateSpace()
    {

        std::vector<std::set<std::shared_ptr<State>>> list_states(getWorld()->getHorizon());
        std::shared_ptr<State> initial_state = getWorld()->getInitialState();
        list_states[0].insert(initial_state);

        for (int i = 0; i < num_sample_states; i++)
        {
            std::shared_ptr<State> current_state = initial_state;

            for (number t = 1; t < getWorld()->getHorizon(); t++)
            {
                current_state = sampleNextState(current_state, t-1);
                list_states[t].insert(current_state);
            }
        }
        for (number t = 0; t < getWorld()->getHorizon(); t++)
        {
            sampled_state_space.push_back(std::make_shared<DiscreteSpace>(tools::set2vector(list_states[t])));
        }
    }

    void PBVI::initStateSpace2()
    {
        for (number t = 0; t < getWorld()->getHorizon(); t++)
        {
            std::set<std::shared_ptr<State>> list_states;
            for (int i = 0; i < num_sample_states; i++)
            {
                list_states.insert(selectOneState(t));
            }
            sampled_state_space.push_back(std::make_shared<DiscreteSpace>(tools::set2vector(list_states)));
        }
    }

    std::shared_ptr<State> PBVI::selectOneState(number t)
    {
        std::shared_ptr<State> current_state = getWorld()->getInitialState();
        for (number h = 0; h < t; h++)
        {
            current_state = sampleNextState(current_state, t);
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