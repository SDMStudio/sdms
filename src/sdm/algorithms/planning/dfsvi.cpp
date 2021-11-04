
#include <sdm/config.hpp>
#include <sdm/algorithms/planning/dfsvi.hpp>
// #include <sdm/world/belief_mdp.hpp>

namespace sdm
{
    DFSVI::DFSVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, double time_max, std::string name)
        : TSVI(world, value_function, error, time_max, name)
    {
    }

    void DFSVI::initLogger()
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

    void DFSVI::logging()
    {
        // Print in loggers some execution variables
        logger->log(trial, getValueFunction()->getValueAt(getWorld()->getInitialState()), getValueFunction()->getSize(), getExecutionTime());
    }

    void DFSVI::initTrial()
    {
        max_error = -std::numeric_limits<double>::max();
        was_updated = false;
    }

    bool DFSVI::stop(const std::shared_ptr<State> &, double, number t)
    {
        return ((max_error <= error && was_updated) || (t >= getWorld()->getHorizon()));
    }

    // SELECT ACTION IN DFSVI
    std::shared_ptr<Space> DFSVI::selectActions(const std::shared_ptr<State> &state, number t)
    {
        return getWorld()->getActionSpaceAt(state, t);
    }

    // SELECT OBSERVATION IN DFSVI
    std::shared_ptr<Space> DFSVI::selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return getWorld()->getObservationSpaceAt(state, action, t);
    }

    // COMPUTE NEXT STATE IN DFSVI
    std::shared_ptr<Space> DFSVI::selectNextStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        std::vector<std::shared_ptr<State>> next_states = {getWorld()->getNextStateAndProba(state, action, observation, t).first};
        return std::make_shared<DiscreteSpace>(next_states);
    }

    void DFSVI::updateValue(const std::shared_ptr<State> &state, number t)
    {
        was_updated = true;
        double old_value = getValueFunction()->getValueAt(state, t);
        TSVI::updateValue(state, t);
        max_error = std::max(max_error, getValueFunction()->getValueAt(state, t) - old_value);
    }

    std::string DFSVI::getAlgorithmName()
    {
        return "DepthFirstSearchVI";
    }

    // void DFSVI::solve()
    // {
    //     do
    //     {
    //         max_error = -std::numeric_limits<double>::max();
    //         value_function_1 = value_function_2->copy();

    //         for (number t = horizon - 1; t >= 0; t--)
    //         {
    //             for (const auto &state : *selectStates(t))
    //             {
    //                 value_function_2->updateValueAt(state->toState(), t);
    //                 max_error = std::max(std::abs(value_function_1->getValueAt(state->toState(), t) - value_function_2->getValueAt(state->toState(), t)), max_error);
    //             }
    //         }
    //     } while !stop();
    // }

}
