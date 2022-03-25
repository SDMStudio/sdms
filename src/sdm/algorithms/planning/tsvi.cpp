
#include <sdm/algorithms/planning/tsvi.hpp>

namespace sdm
{
    TSVI::TSVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, double time_max, std::string name)
        : DynamicProgramming(world, error, name), value_function(value_function), time_max(time_max)
    {
    }

    void TSVI::initialize()
    {
        initLogger();
        value_function->initialize();
    }

    void TSVI::solve()
    {
        printStartInfo();
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
        printEndInfo();

    }

    void TSVI::explore(const std::shared_ptr<State> &state, double cost_so_far, number t)
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
                for (const auto &action : selectActions(state, t))
                {
                    // Select next observation
                    for (const auto &observation : selectObservations(state, action, t))
                    {
                        // Determine the state for a given state, action and observation
                        auto next_state = getWorld()->getNextStateAndProba(state, action, observation, t).first;

                        // Recursive explore
                        explore(next_state, cost_so_far + getWorld()->getDiscount(t) * getWorld()->getReward(state, action, t), t + 1);
                    }
                }
                // Update the value function (backward update)
                this->updateValue(state, t);
            }
        }
        catch (const std::exception &exc)
        {
            // Catch anything thrown within try block that derives from std::exception
            std::cerr << "TSVI::explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void TSVI::updateValue(const std::shared_ptr<State> &state, number t)
    {
        getValueFunction()->updateValueAt(state, t);
    }

    void TSVI::test()
    {
    }

    void TSVI::save()
    {
    }

    std::shared_ptr<ValueFunction> TSVI::getValueFunction()
    {
        return value_function;
    }

    void TSVI::initTrial()
    {
    }

}