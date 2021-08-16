#include <sdm/exception.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>


namespace sdm
{

    double HSVI::TIME_IN_SELECT_STATE = 0,
           HSVI::TIME_IN_SELECT_ACTION = 0,
           HSVI::TIME_INITIALIZATION = 0,
           HSVI::TIME_IN_UPDATE_LB = 0,
           HSVI::TIME_IN_UPDATE_UB = 0,
           HSVI::TIME_IN_PRUNING_LB = 0,
           HSVI::TIME_IN_PRUNING_UB = 0,
           HSVI::TIME_IN_DO_EXCESS =0,
           HSVI::TIME_TO_REMOVE =0;

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
        
        // ************* Precise Logger ****************
        format = "#> Trial :\t{}Horizon :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\t Size_lb({}) \t Size_ub({}) \t Action_Time({}) \t State_Time({}) \t Update_LB_Time({}) \t Update_UB_Time({}) \t Size_JHistories({}) \t Size_IHistory({}) \t Size_ActionSpace ({})  \n";

        // Build a logger that prints logs on the standard output stream
        std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        csv_logger = std::make_shared<sdm::CSVLogger>(this->name_+"_precise_data", std::vector<std::string>{"Trial","Horizon","Error", "Value_LB", "Value_UB", "Size_lb", "Size_ub", "Action_Time","State_Time","Update_LB_Time","Update_UB_Time","Size_JHistories","Size_IHistory","Size_ActionSpace"});

        // Build a multi logger that combines previous loggers
        this->logger_precise_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{csv_logger});
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
        this->duration = 0;
        this->start_time = std::chrono::high_resolution_clock::now();

        do
        {

            // Logging (save data and print algorithms variables)
            //---------------------------------//
            this->current_time = std::chrono::high_resolution_clock::now();
            this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration- HSVI::TIME_TO_REMOVE);
            this->updateTime(current_time,"Time_to_remove");

            //---------------------------------//

            this->do_explore(start_state, 0, 0);

            this->current_time = std::chrono::high_resolution_clock::now();
            this->lower_bound_->do_pruning(this->trial);
            this->updateTime(current_time, "Pruning LB");

            this->current_time = std::chrono::high_resolution_clock::now();
            this->upper_bound_->do_pruning(this->trial);
            this->updateTime(current_time, "Pruning UB");

            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();

            this->trial++;

        } while (!this->do_stop(start_state, 0, 0) && (this->time_max_ >= this->duration-HSVI::TIME_TO_REMOVE));

        //---------------------------------//
        this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration- HSVI::TIME_TO_REMOVE);
        //---------------------------------//
    }

    bool HSVI::do_stop(const std::shared_ptr<State> &s, double cost_so_far, number h)
    {
        return ((this->do_excess(s, cost_so_far, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    void HSVI::do_explore(const std::shared_ptr<State> &state, double cost_so_far, number h)
    {
        std::chrono::high_resolution_clock::time_point start_time_tmp;
        double duration_select_action = 0,duration_next_state = 0,duration_update_lower = 0,duration_update_upper = 0;

        try
        {
            if (!this->do_stop(state, cost_so_far, h))
            {
                if (this->lower_bound_->isInfiniteHorizon())
                {
                    this->lower_bound_->updateValueAt(state, h);
                    this->upper_bound_->updateValueAt(state, h);
                }

// #ifdef LOGTIME
                this->current_time = std::chrono::high_resolution_clock::now();
                start_time_tmp = std::chrono::high_resolution_clock::now();
// #endif

                // Select next action and state following search process
                auto [selected_action, value] = this->world_->selectNextAction(this->lower_bound_, this->upper_bound_, state, h);
// #ifdef LOGTIME
                this->updateTime(current_time,"Action");
                duration_select_action = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
                start_time_tmp = std::chrono::high_resolution_clock::now();
                this->current_time = std::chrono::high_resolution_clock::now();
// #endif
                std::shared_ptr<State> s_ = this->world_->nextState(state, selected_action, h, this->getptr());

// #ifdef LOGTIME
                this->updateTime(current_time,"Next State");
                duration_next_state = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
// #endif

                // Recursive explore
                this->do_explore(s_, cost_so_far + this->world_->getDiscount(h) * this->world_->getReward(state, selected_action, h), h + 1);

// #ifdef LOGTIME
                this->current_time = std::chrono::high_resolution_clock::now();
                start_time_tmp = std::chrono::high_resolution_clock::now();
// #endif

                // Update bounds
                if (((this->trial+1) % this->lb_update_frequency_) == 0)
                {
                    if(keep_same_action_forward_backward_)
                    {
                        this->lower_bound_->updateValueAt(state, selected_action, h);
                    }else
                    {
                        this->lower_bound_->updateValueAt(state, h);
                    }
                }
// #ifdef LOGTIME
                this->updateTime(current_time, "Update Lower");
                duration_update_lower = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
                start_time_tmp = std::chrono::high_resolution_clock::now();
                this->current_time = std::chrono::high_resolution_clock::now();
// #endif

                if (((this->trial+1) % this->ub_update_frequency_) == 0)
                {
                    if(keep_same_action_forward_backward_)
                    {
                        this->upper_bound_->updateValueAt(state, selected_action, h);
                    }else
                    {
                        this->upper_bound_->updateValueAt(state, h);
                    }                
                }
// #ifdef LOGTIME
                this->updateTime(current_time, "Update Upper");
                duration_update_upper = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_tmp).count();
// #endif
            }
            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();

            //---------------DEBUG-----------------//
            if(!state->isBaseItem() && state->getTypeState() == TypeState::OCCUPANCY_STATE)
            {
                this->current_time = std::chrono::high_resolution_clock::now();
                int size_action_space = 0;
                auto action_space = this->world_->getActionSpaceAt(state,h);
                for(const auto&a : *action_space)
                {
                    size_action_space ++;
                }
                //state->toOccupancyState()->getIndividualHistories(std::dynamic_pointer_cast<SerialOccupancyMDP>(this->world_)->getAgentId(h)).size()
                // this->logger_precise_->log(this->trial,h, this->do_excess(state, cost_so_far, h) + this->error_, this->lower_bound_->getValueAt(state), this->upper_bound_->getValueAt(state), this->lower_bound_->getSize(h), this->upper_bound_->getSize(h), duration_select_action, duration_next_state,duration_update_lower,duration_update_upper,state->toOccupancyState()->getJointHistories().size(),state->toOccupancyState()->getIndividualHistories(std::dynamic_pointer_cast<SerialOccupancyMDP>(this->world_)->getAgentId(h)).size(),size_action_space);
                this->updateTime(current_time, "Time_to_remove");
            }

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

            this->updateTime(current_time, "Do_excess");
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

    void HSVI::saveResults(std::string filename, double other)
    {
        std::ofstream ofs;
        ofs.open(filename+ "hsvi_profiling.md", std::ios::out | std::ios::app);
        ofs << "## " << filename << std::endl;
        ofs << "| Trials \t"<< this->trial << std::endl;
        ofs <<"| Error \t"<< this->do_excess(this->start_state, 0, 0) + this->error_ << std::endl;
        ofs <<"| Time \t "<<this->duration<<std::endl;
        ofs <<"| Lower Bound Value \t"<< this->lower_bound_->getValueAt(this->start_state) << std::endl;
        ofs <<"| Upper Bound Value \t"<<this->upper_bound_->getValueAt(this->start_state) << std::endl;
        ofs <<"| Total Size Lower Bound \t"<< this->lower_bound_->getSize() << std::endl;
        ofs <<"| Total Size Upper Bound \t"<< this->upper_bound_->getSize() << std::endl;

        ofs <<"| Horizon \t \t \t| Size Lower Bound \t \t \t| Size Upper Bound \t \t \t "<< std::endl;
        for (size_t i = 0; i < this->planning_horizon_; i++)
        {
            ofs<<"| Horizon \t"<<i<<"|"<<this->lower_bound_->getSize(i)<<"|"<<this->upper_bound_->getSize(i)<< std::endl;
        }
        
        ofs <<"| Number of Node \t"<< std::static_pointer_cast<OccupancyMDP>(this->world_)->getMDPGraph()->getNumNodes() << std::endl;
        number num_max_jhist = 0, tmp;
        for (const auto &state : std::static_pointer_cast<OccupancyMDP>(this->world_)->getStoredStates())
        {
            if (num_max_jhist < (tmp = state->toOccupancyState()->getJointHistories().size()))
            {
                num_max_jhist = tmp;
            }
        }
        ofs <<"| Max number of JHistory \t "<< num_max_jhist << std::endl;
        ofs.close();
    }

    double HSVI::getResult()
    {
        return this->lower_bound_->getValueAt(this->world_->getInitialState());
    }

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
        }else if(information == "Time_to_remove")
        {
            HSVI::TIME_TO_REMOVE += time;
        }
        else if(information == "Pruning LB")
        {
            HSVI::TIME_IN_PRUNING_LB += time;
        }else if(information == "Pruning UB")
        {
            HSVI::TIME_IN_PRUNING_UB += time;
        }
        else if(information == "Do_excess")
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
        HSVI::TIME_IN_DO_EXCESS =0;
        HSVI::TIME_TO_REMOVE =0;
    }
    
} // namespace sdm