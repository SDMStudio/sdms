
#include <sdm/algorithms/planning/value_iteration.hpp>

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

    void ValueIteration::initTrial()
    {
    }

    void ValueIteration::solve()
    {
        printStartInfo();
        startExecutionTime();

        trial = 0;

        do
        {
            initTrial(); // Initialize the trial
            logging();   // Print execution variables in logging output streams
            doTrial();   // Do a trial

            trial++; // At the end of the exploration, go to the next trial

        } while (!stop() && (time_max >= getExecutionTime())); // Do trials until convergence
        logging();                                             // Print execution variables in logging output streams
        printEndInfo();
    }

    void ValueIteration::doTrial()
    {
        try
        {
            for (number t = 0; t < getWorld()->getHorizon(); t++)
            {
                // Select next states
                auto state_space = selectStates(t);
                for (const auto &state : *state_space)
                {
                    // Update the value function (backward update)
                    this->updateValue(state->toState(), t);
                }
            }
        }
        catch (const std::exception &exc)
        {
            // Catch anything thrown within try block that derives from std::exception
            std::cerr << "ValueIteration::explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    bool ValueIteration::stop()
    {
        return (max_error <= error && was_updated);
    }

    void ValueIteration::updateValue(const std::shared_ptr<State> &state, number t)
    {
        tmp_value_function->updateValueAt(state, t);
        was_updated = true;
        max_error = std::max(max_error, getValueFunction()->getValueAt(state, t) - tmp_value_function->getValueAt(state, t));
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

}