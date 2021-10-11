
#include <sdm/algorithms/planning/vi.hpp>

namespace sdm
{
    ValueIteration::ValueIteration(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, double time_max, std::string name)
        : DynamicProgramming(world, error, name), value_function(value_function), time_max(time_max)
    {
    }

    void ValueIteration::initialize()
    {
        initLogger();
        value_function->initialize();
    }
    
    void ValueIteration::printInfo(){
        std::cout << config::LOG_SDMS << "-------------------" << std::endl;
        std::cout << config::LOG_SDMS << "ALGO=ValueIteration"  << std::endl;
        std::cout << config::LOG_SDMS << "# name=" << name << std::endl;
        std::cout << config::LOG_SDMS << "# error="  <<error<< std::endl;
        std::cout << config::LOG_SDMS << "-------------------" << std::endl;
    }

    void ValueIteration::solve()
    {
        printInfo();
        startExecutionTime();

        trial = 0;
        auto initial_state = getWorld()->getInitialState(); // Get the initial node

        do
        {
            initTrial(); // Initialize the trial

            logging(); // Print execution variables in logging output streams

            explore(initial_state, 0, 0); // Explore the tree

            trial++; // At the end of the exploration, go to the next trial

        } while (!stop(initial_state, 0, 0) && (time_max >= getExecutionTime())); // Do trials until convergence
        logging();                                                                // Print execution variables in logging output streams
    }

    void ValueIteration::explore(const std::shared_ptr<State> &state, double cost_so_far, number t)
    {
        try
        {
            if (!stop(state, cost_so_far, t))
            {
                if (value_function->isInfiniteHorizon())
                {
                    // Update the value function (frontward update)
                    updateValue(state, t);
                }

                // Select next action
                auto action_space = selectActions(state, t);
                for (const auto &action : *action_space)
                {
                    // Select next observation
                    auto obs_space = selectObservations(state, action->toAction(), t);
                    for (const auto &observation : *obs_space)
                    {
                        // Select next states
                        auto state_space = selectNextStates(state, action->toAction(), observation->toObservation(), t);
                        for (const auto &next_state : *state_space)
                        {
                            // Determine the state for a given state, action and observation
                            // auto next_state = selectNextState(state, action, observation, t);

                            // Recursive explore
                            explore(next_state->toState(), cost_so_far + getWorld()->getDiscount(t) * getWorld()->getReward(state, action->toAction(), t), t + 1);

                        }
                    }
                }
                // Update the value function (backward update)
                this->updateValue(state, t);
            }
        }
        catch (const std::exception &exc)
        {
            // Catch anything thrown within try block that derives from std::exception
            std::cerr << "ValueIteration::explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ValueIteration::updateValue(const std::shared_ptr<State> &state, number t)
    {
        getValueFunction()->updateValueAt(state, t);
    }

    void ValueIteration::test()
    {
    }

    void ValueIteration::save()
    {
    }

    std::shared_ptr<ValueFunction> ValueIteration::getValueFunction()
    {
        return value_function;
    }

    void ValueIteration::initTrial()
    {
    }

}

// // SELECT ACTION IN HSVI
// std::shared_ptr<Space> ValueIteration::selectActions(const std::shared_ptr<State> &state, number t)
// {
//     return std::make_shared<DiscreteSpace>({upper_bound->getBestAction(state, t)});
// }

// // SELECT ACTION IN HSVI
// std::shared_ptr<Space> ValueIteration::selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
// {
// }

// //  SELECT ACTION IN Perseus
// std::shared_ptr<Space> ValueIteration::selectActions(const std::shared_ptr<State> &state, number t)
// {
//     return std::make_shared<DiscreteSpace>(world->getActionSpaceAt(state, t)->sample(n), false);
// }

// //  SELECT ACTION IN Perseus
// std::shared_ptr<Space> ValueIteration::selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
// {
//     return std::make_shared<DiscreteSpace>(world->getNextObservationDistribution(state, action, t)->sample(n), false);
// }
