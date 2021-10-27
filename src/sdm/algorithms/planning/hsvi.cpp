#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

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
               bool keep_same_action_forward_backward) : ValueIteration(world, lower_bound, error, time_max, name),
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
        ValueIteration::initialize();
        getUpperBound()->initialize();
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


    // SELECT ACTIONS IN HSVI
    std::shared_ptr<Space> HSVI::selectActions(const std::shared_ptr<State> &state, number t)
    {
        std::vector<std::shared_ptr<Action>> selected_actions{getUpperBound()->getGreedyAction(state, t)};
        return std::make_shared<DiscreteSpace>(selected_actions);
    }

    // SELECT OBSERVATION IN HSVI
    std::shared_ptr<Space> HSVI::selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
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
                // selected_next_state = next_state;
            }
        }
        return std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Observation>>{selected_observation});
    }

    // COMPUTE NEXT STATE IN HSVI
    std::shared_ptr<Space> HSVI::selectNextStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        std::vector<std::shared_ptr<State>> selected_next_states{getWorld()->getNextStateAndProba(state, action, observation, t).first};
        return std::make_shared<DiscreteSpace>(selected_next_states);
        // return select_next_state;
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
        // Update lower bounds
        if (((trial + 1) % lb_update_frequency) == 0)
        {
            // if (keep_same_action_forward_backward)
            //     getLowerBound()->updateValueAt(state, selected_action, t);
            // else
            getLowerBound()->updateValueAt(state, t);
        }

        // Update upper bounds
        if (((trial + 1) % ub_update_frequency) == 0)
        {
            // if (keep_same_action_forward_backward)
            //     getUpperBound()->updateValueAt(state, selected_action, t);
            // else
            getUpperBound()->updateValueAt(state, t);
        }
    }

    void HSVI::initLogger()
    {
        // ************* Global Logger ****************
        std::string format = config::LOG_SDMS + "Trial {}\tError :\t{}\t->\tValue_LB({})\tValue_UB({})\t Size_LB({}) \t Size_UB({}) \t Time({})\n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Size_LB", "Size_UB", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
    }

    void HSVI::logging()
    {
        auto initial_state = getWorld()->getInitialState();

        // Print in loggers some execution variables
        logger->log(trial,
                    excess(initial_state, 0, 0) + error,
                    getLowerBound()->getValueAt(initial_state),
                    getUpperBound()->getValueAt(initial_state),
                    getLowerBound()->getSize(),
                    getUpperBound()->getSize(),
                    getExecutionTime());
    }

    void HSVI::initTrial()
    {
        // Do the pruning for the lower bound
        // getLowerBound()->doPruning(trial);

        // Do the pruning for the upper bound
        // getUpperBound()->doPruning(trial);
    }

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

    void HSVI::saveResults(std::string filename, std::string format)
    {
        std::ofstream ofs;
        struct sysinfo memInfo;

        if ((format == ".md"))
        {
            auto initial_state = getWorld()->getInitialState();
            // Compute duration
            duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count();

            auto memory = std::Performance::RanMemoryUsed(memInfo);

            saveParams(filename, format);

            ofs.open(filename + format, std::ios::out | std::ios::app);
            ofs << "## " << filename << "(RESULTS)" << std::endl;

            ofs << " | Time | Trials | Error | LB Value  | UB Value  | Total Size LB | Total Size UB | Num Nodes (oState graph) | Num Nodes (belief graph) | Num Max of JHistory | Memory |" << std::endl;
            ofs << " | ---- | ------ | ----- | --------  | --------  | ------------- | ------------- | ------------------------ | ------------------------ | ------------------- | ------ |" << std::endl;
            ofs << " | " << duration;
            ofs << " | " << trial;
            ofs << " | " << excess(initial_state, 0, 0) + error;
            ofs << " | " << getLowerBound()->getValueAt(initial_state);
            ofs << " | " << getUpperBound()->getValueAt(initial_state);
            ofs << " | " << getLowerBound()->getSize();
            ofs << " | " << getUpperBound()->getSize();
            ofs << " | " << std::dynamic_pointer_cast<OccupancyMDP>(getWorld())->getMDPGraph()->getNumNodes();
            ofs << " | deprecated " ;//<< std::dynamic_pointer_cast<OccupancyMDP>(getWorld())->getUnderlyingBeliefMDP()->getMDPGraph()->getNumNodes();

            number num_max_jhist = 0, tmp;
            for (const auto &state : std::dynamic_pointer_cast<OccupancyMDP>(getWorld())->getStoredStates())
            {
                if (num_max_jhist < (tmp = state->toOccupancyState()->getJointHistories().size()))
                {
                    num_max_jhist = tmp;
                }
            }
            ofs << " | " << num_max_jhist;
            ofs << " | " << memory;
            ofs << " | " << std::endl
                << std::endl;
            ofs.close();
        }
    }
} // namespace sdm