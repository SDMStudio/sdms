
#include <sdm/algorithms/planning/perseus.hpp>

namespace sdm
{
    Perseus::Perseus(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, double time_max, std::string name)
        : ValueIteration(world, value_function, error, time_max, name)
    {
    }

    void Perseus::initLogger()
    {
        // ************* Global Logger ****************
        std::string format = config::LOG_SDMS + "Trial {:<8} Error {:<12.4f} Value {:<12.4f} Size {:<10} Time {:<12.4f}\n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Size_LB", "Size_UB", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
    }

    void Perseus::logging()
    {
        auto initial_state = getWorld()->getInitialState();

        // Print in loggers some execution variables
        logger->log(trial,
                    0,
                    getValueFunction()->getValueAt(initial_state),
                    getValueFunction()->getSize(),
                    getExecutionTime());
    }

    std::shared_ptr<State> Perseus::selectOneState(number t)
    {
        std::shared_ptr<State> current_state = getWorld()->getInitialState();
        std::shared_ptr<Action> current_action;
        std::shared_ptr<Observation> current_observation;
        for (number h = 0; h < t; h++)
        {
            current_action = getWorld()->getRandomAction(current_state, t);
            current_observation = getWorld()->getObservationSpaceAt(current_state, current_action, t)->sample()->toObservation();
            current_state = getWorld()->getNextStateAndProba(current_state, current_action, current_observation, t).first;
        }
        return current_state;
    }

    std::shared_ptr<Space> Perseus::selectStates(number t)
    {
        if (isInstanceOf<MDPInterface>(this->getWorld()))
        {
            return std::dynamic_pointer_cast<MDPInterface>(getWorld())->getStateSpace();
        }
        else
        {
            std::vector<std::shared_ptr<State>> list_states;
            for (int i = 0; i < NumExploreState[t]; i++)
            {
                list_states.push_back(this->selectOneState(t));
            }
            return std::make_shared<DiscreteSpace>(list_states);
        }
    }

    std::string Perseus::getAlgorithmName()
    {
        return "Perseus";
    }

}