
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
        tmp_value_function->initialize();
    }

    void ValueIteration::initTrial()
    {
        max_error = -std::numeric_limits<double>::infinity();
        was_updated = false;
        value_function = tmp_value_function->copy()->toValueFunction();
    }

    void ValueIteration::initLogger()
    {
        // ************* Global Logger ****************
        std::string format = config::LOG_SDMS + "Trial {:<8} Value {:<12.4f} Size {:<10} Time {:<12.4f}\n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name, std::vector<std::string>{"Trial", "Value", "Size", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
    }

    void ValueIteration::logging()
    {
        auto initial_state = getWorld()->getInitialState();

        // Print in loggers some execution variables
        this->logger->log(trial,
                          getValueFunction()->getValueAt(initial_state),
                          getValueFunction()->getSize(),
                          getExecutionTime());
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

        getTmpValueFunction()->updateValueAt(state, t);
        was_updated = true;
        max_error = std::max(max_error, std::abs(getValueFunction()->getValueAt(state, t) - getTmpValueFunction()->getValueAt(state, t)));
    }

    std::shared_ptr<Space> ValueIteration::selectStates(number t)
    {
        return getWorld()->getUnderlyingProblem()->getStateSpace(t);
    }

    void ValueIteration::test()
    {
    }

    void ValueIteration::save()
    {
    }

    std::shared_ptr<ValueFunction> ValueIteration::getValueFunction()
    {
        return this->value_function;
    }

    std::shared_ptr<ValueFunction> ValueIteration::getTmpValueFunction()
    {
        return this->tmp_value_function;
    }

    void ValueIteration::setTmpValueFunction(const std::shared_ptr<ValueFunction> &tmp_vf)
    {
        this->tmp_value_function = tmp_vf;
    }

    std::string ValueIteration::getAlgorithmName()
    {
        return "ValueIteration";
    }
}