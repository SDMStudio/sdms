#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/utils/value_function/prunable_structure.hpp>
#include <sdm/utils/value_function/vfunction/sawtooth_value_function.hpp>
#include <sdm/utils/value_function/initializer/pomdp_relaxation.hpp>

namespace sdm
{
    HSVI::HSVI(std::shared_ptr<SolvableByHSVI> &world,
               std::shared_ptr<ValueFunction> lower_bound,
               std::shared_ptr<ValueFunction> upper_bound,
               double error,
               number num_max_trials,
               std::string name,
               number lb_update_frequency,
               number ub_update_frequency,
               double time_max,
               bool keep_same_action_forward_backward) : TSVI(world, lower_bound, error, time_max, name),
                                                         lower_bound(lower_bound),
                                                         upper_bound(upper_bound),
                                                         num_max_trials(num_max_trials),
                                                         lb_update_frequency(lb_update_frequency),
                                                         ub_update_frequency(ub_update_frequency),
                                                         keep_same_action_forward_backward(keep_same_action_forward_backward)
    {
    }

    void HSVI::initialize()
    {
        TSVI::initialize();
        getUpperBound()->initialize();

        std::cout << this->getWorld()->getInitialState()->str() << std::endl;
        std::cout << this->getUpperBound()->getValueAt(this->getWorld()->getInitialState()) << std::endl;
    }

    bool HSVI::stop(const std::shared_ptr<State> &state, double cost_so_far, number t)
    {
        return ((excess(state, cost_so_far, t) <= 0) || (trial > num_max_trials));
    }

    double HSVI::excess(const std::shared_ptr<State> &state, double cost_so_far, number t)
    {
        double value_excess;
        try
        {
            value_excess = getWorld()->do_excess(getLowerBound()->getValueAt(getWorld()->getInitialState()), getLowerBound()->getValueAt(state, t), getUpperBound()->getValueAt(state, t), cost_so_far, error, t);
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "HSVI::excess(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
        return value_excess;
    }

    void HSVI::explore(const std::shared_ptr<State> &state, double cost_so_far, number t)
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
                auto [action, value] = getUpperBound()->getGreedyActionAndValue(state, t);

                // Select next observation
                for (const auto &observation : selectObservations(state, action, t))
                {
                    // Determine the state for a given state, action and observation
                    auto next_state = getWorld()->getNextStateAndProba(state, action, observation, t).first;
                    // Recursive explore
                    explore(next_state, cost_so_far + getWorld()->getDiscount(t) * getWorld()->getReward(state, action, t), t + 1);
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

    // SELECT ACTIONS IN HSVI
    std::vector<std::shared_ptr<Action>> HSVI::selectActions(const std::shared_ptr<State> &state, number t)
    {
        return {getUpperBound()->getGreedyAction(state, t)};
    }

    // SELECT OBSERVATION IN HSVI
    std::vector<std::shared_ptr<Observation>> HSVI::selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        double error, biggest_error = -std::numeric_limits<double>::max();
        std::shared_ptr<Observation> selected_observation;

        // Select next observation
        auto observation_space = getWorld()->getObservationSpaceAt(state, action->toAction(), t);
        for (const auto &observation : *observation_space)
        {
            // Get the next state and probability
            auto [next_state, transition_proba] = getWorld()->getNextStateAndProba(state, action->toAction(), observation->toObservation(), t);

            // Compute error correlated to this next state
            error = transition_proba * excess(next_state, 0, t + 1);
            if (error > biggest_error)
            {
                biggest_error = error;
                selected_observation = observation->toObservation();
            }
        }
        return {selected_observation};
    }

    std::shared_ptr<ValueFunction> HSVI::getLowerBound() const
    {
        return lower_bound;
    }

    std::shared_ptr<ValueFunction> HSVI::getUpperBound() const
    {
        return upper_bound;
    }

    void HSVI::updateValue(const std::shared_ptr<State> &state, number t)
    {
        // auto [action, value] = this->getUpperBound()->getGreedyActionAndValue(state, t);
        this->getUpperBound()->getUpdateOperator()->update(state, /* value, */ t);
        this->getLowerBound()->getUpdateOperator()->update(state, /* action, */ t);
    }

    void HSVI::initTrial()
    {
        // Do the pruning for the lower bound
        if (auto prunable_vf = std::dynamic_pointer_cast<PrunableStructure>(getLowerBound()))
            prunable_vf->doPruning(trial);

        // Do the pruning for the upper bound
        if (auto prunable_vf = std::dynamic_pointer_cast<PrunableStructure>(getUpperBound()))
            prunable_vf->doPruning(trial);
    }

    void HSVI::initLogger()
    {
        // ************* Global Logger ****************
        // Text Format for standard output stream
        std::string format = "\r" + config::LOG_SDMS + "Trial {:<8} Error {:<12.4f} Value_LB {:<12.4f} Value_UB {:<12.4f} Size_LB {:<10} Size_UB {:<10} Time {:<12.4f}";

        // Titles of logs
        std::vector<std::string> list_logs{"Trial", "Error", "Value_LB", "Value_UB", "Size_LB", "Size_UB", "Time"};

        // Specific logs for belief MDPs
        if (sdm::isInstanceOf<BeliefMDPInterface>(getWorld()))
        {
            format = format + " NumState {:<8}";
            list_logs.push_back("NumState");
        }
        format = format + "";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name, list_logs);

        // Build a multi logger that combines previous loggers
        this->logger = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
    }

    void HSVI::logging()
    {
        auto initial_state = getWorld()->getInitialState();

        std::cout << initial_state->str() << std::endl;
        std::cout << this->getLowerBound()->str() << std::endl;
        std::cout << this->getUpperBound()->str() << std::endl;
        std::cout << this->getLowerBound()->getValueAt(initial_state) << std::endl;
        std::cout << this->getUpperBound()->getValueAt(initial_state) << std::endl;

        if (auto derived = std::dynamic_pointer_cast<BeliefMDPInterface>(getWorld()))
        {
            // Print in loggers some execution variables
            logger->log(trial,
                        excess(initial_state, 0, 0) + error,
                        getLowerBound()->getValueAt(initial_state),
                        getUpperBound()->getValueAt(initial_state),
                        getLowerBound()->getSize(),
                        getUpperBound()->getSize(),
                        getExecutionTime(),
                        derived->getMDPGraph()->getNumNodes());
        }
        else
        {
            // Print in loggers some execution variables
            logger->log(trial,
                        excess(initial_state, 0, 0) + error,
                        getLowerBound()->getValueAt(initial_state),
                        getUpperBound()->getValueAt(initial_state),
                        getLowerBound()->getSize(),
                        getUpperBound()->getSize(),
                        getExecutionTime());
        }
    }

    std::string HSVI::getAlgorithmName() { return "HSVI"; }

    void HSVI::saveParams(std::string filename, std::string format)
    {
        std::ofstream ofs;
        ofs.open(filename + format, std::ios::out | std::ios::app);

        if ((format == ".md"))
        {
            ofs << "## " << filename << "(PARAMS)" << std::endl;

            ofs << " | MAX_TRIAL | MAX_TIME | Error |  Horizon | p_o  | p_b | p_c | " << std::endl;
            ofs << " | --------- | -------- | ----- |  ------- | ---  | --- | --- | " << std::endl;
            ofs << " | " << num_max_trials;
            ofs << " | " << time_max;
            ofs << " | " << error;
            ofs << " | " << getWorld()->getHorizon();
            ofs << " | " << OccupancyState::PRECISION;
            ofs << " | " << Belief::PRECISION;
            ofs << " | " << PrivateOccupancyState::PRECISION_COMPRESSION;
            ofs << " | " << std::endl
                << std::endl;
        }
        ofs.close();
    }

} // namespace sdm