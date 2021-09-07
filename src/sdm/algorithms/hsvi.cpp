#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

namespace sdm
{
#ifdef LOGTIME

    double HSVI::TIME_IN_SELECT_STATE = 0,
           HSVI::TIME_IN_SELECT_ACTION = 0,
           HSVI::TIME_INITIALIZATION = 0,
           HSVI::TIME_IN_UPDATE_LB = 0,
           HSVI::TIME_IN_UPDATE_UB = 0,
           HSVI::TIME_IN_PRUNING_LB = 0,
           HSVI::TIME_IN_PRUNING_UB = 0,
           HSVI::TIME_IN_DO_EXCESS = 0;
#endif

    double HSVI::TIME_TO_REMOVE = 0;

    HSVI::HSVI(std::shared_ptr<SolvableByHSVI> &world,
               std::shared_ptr<ValueFunction> lower_bound,
               std::shared_ptr<ValueFunction> upper_bound,
               number planning_horizon,
               double error,
               number num_max_trials,
               std::string name,
               number lb_update_frequency,
               number ub_update_frequency,
               double time_max,
               bool keep_same_action_forward_backward) : world_(world),
                                                         lower_bound_(lower_bound),
                                                         upper_bound_(upper_bound),
                                                         error_(error),
                                                         time_max_(time_max),
                                                         planning_horizon_(planning_horizon),
                                                         name_(name),
                                                         lb_update_frequency_(lb_update_frequency),
                                                         ub_update_frequency_(ub_update_frequency),
                                                         keep_same_action_forward_backward_(keep_same_action_forward_backward)
    {
        this->MAX_TRIALS = num_max_trials;
    }

    std::shared_ptr<HSVI> HSVI::getptr()
    {
        return this->shared_from_this();
    }

    void HSVI::initLogger()
    {

        // ************* Global Logger ****************
        std::string format = "#> Trial :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\t Size_lower_bound({}) \t Size_upper_bound({}) \t Time({})  \n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that prints logs in a file
        // auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Size_lower_bound", "Size_upper_bound", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});

#ifdef LOGTIME
        // ************* Precise Logger ****************
        format = "#> Trial :\t{}Horizon :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\t Size_lb({}) \t Size_ub({}) \t Action_Time({}) \t State_Time({}) \t Update_LB_Time({}) \t Update_UB_Time({}) \t Size_JHistories({}) \t Size_IHistory({}) \t Size_ActionSpace ({})  \n";

        // Build a logger that prints logs on the standard output stream
        std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        csv_logger = std::make_shared<sdm::CSVLogger>(this->name_ + "_precise_data", std::vector<std::string>{"Trial", "Horizon", "Error", "Value_LB", "Value_UB", "Size_lb", "Size_ub", "Action_Time", "State_Time", "Update_LB_Time", "Update_UB_Time", "Size_JHistories", "Size_IHistory", "Size_ActionSpace"});

        // Build a multi logger that combines previous loggers
        this->logger_precise_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{csv_logger});
#endif
    }

    void HSVI::do_initialize()
    {
        this->initLogger();
        this->lower_bound_->initialize();
        this->upper_bound_->initialize();
    }

    void HSVI::do_solve()
    {
        std::cout << "\n\n###############################################################\n";
        std::cout << "#############    Start HSVI \"" << this->name_ << "\"    ####################\n";
        std::cout << "###############################################################\n\n";

        this->start_state = this->world_->getInitialState();

        this->trial = 0;

        this->duration = 0.0;
        this->start_time = std::chrono::high_resolution_clock::now();

        do
        {
            this->current_time = std::chrono::high_resolution_clock::now();
            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(this->current_time - this->start_time).count();
            this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration - HSVI::TIME_TO_REMOVE);
#ifdef LOGTIME
            this->updateTime(current_time, "Time_to_remove");
#endif

            //Explore the tree
            this->do_explore(start_state, 0, 0);

#ifdef LOGTIME
            this->current_time = std::chrono::high_resolution_clock::now();
#endif

            //Do the pruning for the lower bound
            this->lower_bound_->do_pruning(this->trial);

#ifdef LOGTIME
            this->updateTime(current_time, "Pruning LB");
            this->current_time = std::chrono::high_resolution_clock::now();
#endif

            //DO the pruning for the upper bound
            this->upper_bound_->do_pruning(this->trial);

#ifdef LOGTIME
            this->updateTime(current_time, "Pruning UB");
            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();
#endif

            this->trial++;

        } while (!this->do_stop(start_state, 0, 0) && (this->time_max_ >= this->duration - HSVI::TIME_TO_REMOVE));

        //---------------------------------//
#ifdef LOGTIME
        this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration - HSVI::TIME_TO_REMOVE);
#endif
        //---------------------------------//
    }

    bool HSVI::do_stop(const std::shared_ptr<State> &s, double cost_so_far, number h)
    {
        return ((this->do_excess(s, cost_so_far, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    void HSVI::do_explore(const std::shared_ptr<State> &state, double cost_so_far, number h)
    {
#ifdef LOGTIME
        std::chrono::high_resolution_clock::time_point start_time_tmp;
        double duration_select_action = 0.0, duration_next_state = 0.0, duration_update_lower = 0.0, duration_update_upper = 0.0;
#endif
        struct sysinfo memInfo;

        try
        {
            if (!this->do_stop(state, cost_so_far, h))
            {
                if (this->lower_bound_->isInfiniteHorizon())
                {
                    this->lower_bound_->updateValueAt(state, h);
                    this->upper_bound_->updateValueAt(state, h);
                }

#ifdef LOGTIME
                this->current_time = std::chrono::high_resolution_clock::now();
                start_time_tmp = std::chrono::high_resolution_clock::now();
#endif

                long long current_mem_used = std::Performance::RanMemoryUsed(memInfo);

                // Select next action and state following search process
                auto [selected_action, value] = this->world_->selectNextAction(this->lower_bound_, this->upper_bound_, state, h);

                long long new_mem_used = std::Performance::RanMemoryUsed(memInfo), total_mem = std::Performance::totalMemory(memInfo);
                // std::cout << "Memory (selectAction) : " << 100*((new_mem_used / 1.e6) / (total_mem / 1.e6)) << "% | diff(" << (new_mem_used/1.e6 - current_mem_used/1.e6) << ")" << std::endl;
                current_mem_used = new_mem_used;

#ifdef LOGTIME
                this->updateTime(current_time, "Action");
                duration_select_action = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
                start_time_tmp = std::chrono::high_resolution_clock::now();
                this->current_time = std::chrono::high_resolution_clock::now();
#endif
                //Determine the state thanks to the action
                std::shared_ptr<State> s_ = this->world_->nextState(state, selected_action, h, this->getptr());

                new_mem_used = std::Performance::RanMemoryUsed(memInfo), total_mem = std::Performance::totalMemory(memInfo);
                // std::cout << "Memory (nextState) : " << 100*((new_mem_used / 1.e6) / (total_mem / 1.e6)) << "% | diff(" << (new_mem_used/1.e6 - current_mem_used/1.e6) << ")" << std::endl;
                current_mem_used = new_mem_used;

#ifdef LOGTIME
                this->updateTime(current_time, "Next State");
                duration_next_state = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
#endif

                // Recursive explore
                this->do_explore(s_, cost_so_far + this->world_->getDiscount(h) * this->world_->getReward(state, selected_action, h), h + 1);

#ifdef LOGTIME
                this->current_time = std::chrono::high_resolution_clock::now();
                start_time_tmp = std::chrono::high_resolution_clock::now();
#endif

                // Update lower bounds
                if (((this->trial + 1) % this->lb_update_frequency_) == 0)
                {
                    if (keep_same_action_forward_backward_)
                    {
                        this->lower_bound_->updateValueAt(state, selected_action, h);
                    }
                    else
                    {
                        this->lower_bound_->updateValueAt(state, h);
                    }
                }

                new_mem_used = std::Performance::RanMemoryUsed(memInfo), total_mem = std::Performance::totalMemory(memInfo);
                // std::cout << "Memory (updateLB) : " << 100*((new_mem_used / 1.e6) / (total_mem / 1.e6)) << "% | diff(" << (new_mem_used/1.e6 - current_mem_used/1.e6) << ")" << std::endl;
                current_mem_used = new_mem_used;
#ifdef LOGTIME
                this->updateTime(current_time, "Update Lower");
                duration_update_lower = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
                start_time_tmp = std::chrono::high_resolution_clock::now();
                this->current_time = std::chrono::high_resolution_clock::now();
#endif

                // Update upper bounds
                if (((this->trial + 1) % this->ub_update_frequency_) == 0)
                {
                    if (keep_same_action_forward_backward_)
                    {
                        this->upper_bound_->updateValueAt(state, selected_action, h);
                    }
                    else
                    {
                        this->upper_bound_->updateValueAt(state, h);
                    }
                }

                new_mem_used = std::Performance::RanMemoryUsed(memInfo), total_mem = std::Performance::totalMemory(memInfo);
                // std::cout << "Memory (updateUB) : " << 100*((new_mem_used / 1.e6) / (total_mem / 1.e6)) << "% | diff(" << (new_mem_used/1.e6 - current_mem_used/1.e6) << ")" << std::endl;
                current_mem_used = new_mem_used;
#ifdef LOGTIME
                this->updateTime(current_time, "Update Upper");
                duration_update_upper = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
#endif
            }
            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();

            //---------------DEBUG-----------------//
#ifdef LOGTIME
            if (!state->isBaseItem() && state->getTypeState() == TypeState::OCCUPANCY_STATE)
            {
                this->current_time = std::chrono::high_resolution_clock::now();

                //Determine the number of decision rule
                int size_action_space = 0;
                auto action_space = this->world_->getActionSpaceAt(state, h);
                for (const auto &a : *action_space)
                {
                    size_action_space++;
                }

                //Add this information to the logger
                this->logger_precise_->log(this->trial, h, this->do_excess(state, cost_so_far, h) + this->error_, this->lower_bound_->getValueAt(state), this->upper_bound_->getValueAt(state), this->lower_bound_->getSize(h), this->upper_bound_->getSize(h), duration_select_action, duration_next_state, duration_update_lower, duration_update_upper, state->toOccupancyState()->getJointHistories().size(), 0, size_action_space);
                this->updateTime(current_time, "Time_to_remove");
            }
#endif
            // std::cout << "\t\t#>s h:" << h << "\t V_lb(" << this->lower_bound_->getValueAt(state, h) << ")\tV_ub(" << this->upper_bound_->getValueAt(state, h) << ")" << std::endl;
            //-----------------DEBUG----------------//

            // ------------- TEST ------------
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "HSVI::do_explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    double HSVI::do_excess(const std::shared_ptr<State> &s, double cost_so_far, number h)
    {
        try
        {
            this->current_time = std::chrono::high_resolution_clock::now();

            const auto &lb = this->lower_bound_->getValueAt(s, h);
            const auto &ub = this->upper_bound_->getValueAt(s, h);
            const auto &incumbent = this->lower_bound_->getValueAt(this->world_->getInitialState());
            double value_do_excess = this->world_->do_excess(incumbent, lb, ub, cost_so_far, this->error_, h);

#ifdef LOGTIME
            this->updateTime(current_time, "Do_excess");
#endif
            return value_do_excess;
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "HSVI::do_excess(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void HSVI::do_test()
    {
        std::shared_ptr<State> ostate = this->world_->getInitialState();
        std::shared_ptr<Action> jdr;
        number end = (this->planning_horizon_ > 0) ? this->planning_horizon_ : 10;
        for (number i = 0; i < end; i++)
        {
            std::cout << "\n------------------------\nTIMESTEP " << i << "\n------------------------\n"
                      << std::endl;
            jdr = this->lower_bound_->getBestAction(ostate, i);
            std::cout << "#> State\n"
                      << ostate->str() << "\n"
                      << std::endl;
            std::cout << "#> Action\n"
                      << jdr->str() << std::endl;
            ostate = this->world_->nextState(ostate, jdr, i, this->getptr());
        }
    }

    void HSVI::do_save()
    {
        this->getLowerBound()->save(this->name_ + "_lb");
    }

    std::shared_ptr<ValueFunction> HSVI::getLowerBound() const
    {
        return this->lower_bound_;
    }

    std::shared_ptr<ValueFunction> HSVI::getUpperBound() const
    {
        return this->upper_bound_;
    }

    int HSVI::getTrial()
    {
        return this->trial;
    }

    void HSVI::saveParams(std::string filename, std::string format)
    {
        std::ofstream ofs;
        ofs.open(filename + format, std::ios::out | std::ios::app);

        if ((format == ".md"))
        {
            ofs << "## " << filename << "(PARAMS)" << std::endl;

            ofs << " | MAX_TRIAL | MAX_TIME | Error | Discount  | Horizon | p_o  | p_b | p_c | " << std::endl;
            ofs << " | --------- | -------- | ----- | --------  | ------- | ---  | --- | --- | " << std::endl;
            ofs << " | " << this->MAX_TRIALS;
            ofs << " | " << this->time_max_;
            ofs << " | " << this->error_;
            ofs << " | " << this->planning_horizon_;
            ofs << " | " << OccupancyState::PRECISION;
            ofs << " | " << Belief::PRECISION;
            ofs << " | " << 0;
            ofs << " | " << std::endl
                << std::endl;
        }
        ofs.close();
    }

    void HSVI::saveResults(std::string filename, std::string format)
    {
        std::ofstream ofs;
        ofs.open(filename + format, std::ios::out | std::ios::app);

        if ((format == ".md"))
        {
            // Compute duration
            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();

            // this->saveParams(filename, format);

            ofs << "## " << filename << "(RESULTS)" << std::endl;

            ofs << " | Time | Trials | Error | LB Value  | UB Value  | Total Size LB | Total Size UB | Num Nodes (oState graph) | Num Nodes (belief graph) | Num Max of JHistory | " << std::endl;
            ofs << " | ---- | ------ | ----- | --------  | --------  | ------------- | ------------- | ------------------------ | ------------------------ | ------------------- | " << std::endl;
            ofs << " | " << this->duration;
            ofs << " | " << this->trial;
            ofs << " | " << this->do_excess(this->start_state, 0, 0) + this->error_;
            ofs << " | " << this->lower_bound_->getValueAt(this->start_state);
            ofs << " | " << this->upper_bound_->getValueAt(this->start_state);
            ofs << " | " << this->lower_bound_->getSize();
            ofs << " | " << this->upper_bound_->getSize();
            ofs << " | " << std::static_pointer_cast<OccupancyMDP>(this->world_)->getMDPGraph()->getNumNodes();
            ofs << " | " << std::static_pointer_cast<OccupancyMDP>(this->world_)->getUnderlyingBeliefMDP()->getMDPGraph()->getNumNodes();

            number num_max_jhist = 0, tmp;
            for (const auto &state : std::static_pointer_cast<OccupancyMDP>(this->world_)->getStoredStates())
            {
                if (num_max_jhist < (tmp = state->toOccupancyState()->getJointHistories().size()))
                {
                    num_max_jhist = tmp;
                }
            }
            ofs << " | " << num_max_jhist;
            ofs << " | " << std::endl
                << std::endl;
        }
        ofs.close();
    }

    double HSVI::getResult()
    {
        return this->lower_bound_->getValueAt(this->world_->getInitialState());
    }

#ifdef LOGTIME
    void HSVI::updateTime(std::chrono::high_resolution_clock::time_point start_time, std::string information)
    {
        auto time = std::Performance::computeTime(start_time);

        if (information == "Action")
        {
            HSVI::TIME_IN_SELECT_ACTION += time;
        }
        else if (information == "Update Lower")
        {
            HSVI::TIME_IN_UPDATE_LB += time;
        }
        else if (information == "Update Upper")
        {
            HSVI::TIME_IN_UPDATE_UB += time;
        }
        else if (information == "Next State")
        {
            HSVI::TIME_IN_SELECT_STATE += time;
        }
        else if (information == "Intialisation")
        {
            // HSVI::TIME_IN_SELECT_STATE += time;
        }
        else if (information == "Time_to_remove")
        {
            HSVI::TIME_TO_REMOVE += time;
        }
        else if (information == "Pruning LB")
        {
            HSVI::TIME_IN_PRUNING_LB += time;
        }
        else if (information == "Pruning UB")
        {
            HSVI::TIME_IN_PRUNING_UB += time;
        }
        else if (information == "Do_excess")
        {
            HSVI::TIME_IN_DO_EXCESS += time;
        }
    }

    void HSVI::cleanTIME()
    {
        HSVI::TIME_IN_SELECT_STATE = 0;
        HSVI::TIME_IN_SELECT_ACTION = 0;
        HSVI::TIME_INITIALIZATION = 0;
        HSVI::TIME_IN_UPDATE_LB = 0;
        HSVI::TIME_IN_UPDATE_UB = 0;
        HSVI::TIME_IN_PRUNING_LB = 0;
        HSVI::TIME_IN_PRUNING_UB = 0;
        HSVI::TIME_IN_DO_EXCESS = 0;
        HSVI::TIME_TO_REMOVE = 0;
    }
#endif
} // namespace sdm