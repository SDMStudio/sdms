#include <sdm/exception.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{

    double HSVI::TIME_IN_SELECT_STATE = 0,
           HSVI::TIME_IN_SELECT_ACTION = 0,
           HSVI::TIME_INITIALIZATION = 0,
           HSVI::TIME_IN_UPDATE_LB = 0,
           HSVI::TIME_IN_UPDATE_UB = 0;

    HSVI::HSVI(std::shared_ptr<SolvableByHSVI> &world,
               std::shared_ptr<ValueFunction> lower_bound,
               std::shared_ptr<ValueFunction> upper_bound,
               number planning_horizon,
               double error,
               number num_max_trials,
               std::string name,
               number lb_update_frequency,
               number ub_update_frequency,
               double time_max) : world_(world),
                                  lower_bound_(lower_bound),
                                  upper_bound_(upper_bound),
                                  error_(error),
                                  time_max_(time_max),
                                  planning_horizon_(planning_horizon),
                                  name_(name),
                                  lb_update_frequency_(lb_update_frequency),
                                  ub_update_frequency_(ub_update_frequency)
    {
        this->MAX_TRIALS = num_max_trials;
    }

    std::shared_ptr<HSVI> HSVI::getptr()
    {
        return this->shared_from_this();
    }

    void HSVI::initLogger()
    {
        std::string format = "#> Trial :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\t Size_lower_bound({}) \t Size_upper_bound({}) \t Time({})  \n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that prints logs in a file
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Size_lower_bound", "Size_upper_bound", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    void HSVI::do_initialize()
    {
        clock_t t_begin = clock();
        this->initLogger();

        this->lower_bound_->initialize();
        this->upper_bound_->initialize();
        HSVI::TIME_INITIALIZATION += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
    }

    void HSVI::do_solve()
    {
        std::cout << "\n\n###############################################################\n";
        std::cout << "#############    Start HSVI \"" << this->name_ << "\"    ####################\n";
        std::cout << "###############################################################\n\n";

        this->start_state = this->world_->getInitialState();

        this->trial = 0;
        this->start_time = std::chrono::high_resolution_clock::now();
        this->current_time = this->start_time;
        this->duration = 0;

        do
        {
// #ifdef LOGTIME
            // this->printTime();
// #endif

            // Logging (save data and print algorithms variables)
            //---------------------------------//
            this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration);
            //---------------------------------//

            this->do_explore(start_state, 0, 0);

            // if(this->trial >0)
            // {
            //     std::cout<<this->upper_bound_->str()<<std::endl;
            // }

            this->lower_bound_->do_pruning(this->trial);
            this->upper_bound_->do_pruning(this->trial);

            this->current_time = std::chrono::high_resolution_clock::now();
            this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(this->current_time - this->start_time).count();

            this->trial++;

        } while (!this->do_stop(start_state, 0, 0) && (this->time_max_ >= this->duration));

        //---------------------------------//
        this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration);
        // std::cout << "Final LB : \n" << this->lower_bound_->str() << "Final UB : \n" << this->upper_bound_->str() << std::endl;
        //---------------------------------//
    }

    bool HSVI::do_stop(const std::shared_ptr<State> &s, double cost_so_far, number h)
    {
        // if(h==1)
        // {
        //     std::cout<<"DO excess "<<this->do_excess(s, cost_so_far, h)<<std::endl;
        //     // std::cout<<"state "<<s->str()<<std::endl;
        // }
        return ((this->do_excess(s, cost_so_far, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    void HSVI::do_explore(const std::shared_ptr<State> &state, double cost_so_far, number h)
    {
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
                this->StartTime();
// #endif

                // Select next action and state following search process
                auto [selected_action, value] = this->world_->selectNextAction(this->lower_bound_, this->upper_bound_, state, h);
// #ifdef LOGTIME
                this->updateTime("Action");
                this->StartTime();
// #endif
                std::shared_ptr<State> s_ = this->world_->nextState(state, selected_action, h, this->getptr());

// #ifdef LOGTIME
                this->updateTime("Next State");
// #endif

                // Recursive explore
                this->do_explore(s_, cost_so_far + this->world_->getDiscount(h) * this->world_->getReward(state, selected_action, h), h + 1);

// #ifdef LOGTIME
                this->StartTime();
// #endif

                // Update bounds
                if (((this->trial+1) % this->lb_update_frequency_) == 0)
                {
                    this->lower_bound_->updateValueAt(state, h);
                }
// #ifdef LOGTIME
                this->updateTime("Update Lower");
                this->StartTime();
// #endif

                if (((this->trial+1) % this->ub_update_frequency_) == 0)
                {
                    this->upper_bound_->updateValueAt(state, h);
                }
// #ifdef LOGTIME
                this->updateTime("Update Upper");
// #endif
            }

            //---------------DEBUG-----------------//
            // std::cout << "\t\t#>s h:" << h << "\t V_lb(" << this->lower_bound_->getValueAt(s, h) << ")\tV_ub(" << this->upper_bound_->getValueAt(s, h) << ")" << std::endl;
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
            const auto &lb = this->lower_bound_->getValueAt(s, h);
            const auto &ub = this->upper_bound_->getValueAt(s, h);
            const auto &incumbent = this->lower_bound_->getValueAt(this->world_->getInitialState());

            // if(h==1)
            // {
            //     std::cout<<"cost_so_far"<<cost_so_far<<std::endl;

            //     std::cout<<"Do excess : ub "<<ub<<", lb "<<lb<<std::endl;
            //     // std::cout<<"State Compressed"<<s->str()<<std::endl;
                
            //     if(lb>ub)
            //     {
            //         exit(-1);
            //     }
            // }

            // if(h==1)
            // {
            //     switch (s->getTypeState())
            //     {
            //     case TypeState::OCCUPANCY_STATE:
            //         std::cout<<"State UnCompressed"<<s->toOccupancyState()->getOneStepUncompressedOccupancy()->str()<<std::endl;
            //         break;
                
            //     default:
            //         break;
            //     }
            // }

            return this->world_->do_excess(incumbent, lb, ub, cost_so_far, this->error_, h);
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
        auto interactive_env = std::dynamic_pointer_cast<GymInterface>(this->world_);
        std::shared_ptr<Observation> ostate = interactive_env->reset();
        std::shared_ptr<Action> jdr;
        number end = (this->planning_horizon_ > 0) ? this->planning_horizon_ : 10;
        for (number t = 0; t < end; t++)
        {
            std::cout << "\n------------------------\nTIMESTEP " << t << "\n------------------------\n"
                      << std::endl;
            jdr = this->lower_bound_->getBestAction(ostate->toState(), t);
            std::cout << "#> State" << std::endl;
            std::cout << ostate->str() << std::endl;
            std::cout << "#> Action" << std::endl;
            std::cout << jdr->str() << std::endl;
            auto [tmp_ostate, rew, _] = interactive_env->step(jdr);
            ostate = tmp_ostate;
            std::cout << "Reward : " << rew << std::endl;
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
        ofs.open(filename, std::ios::out | std::ios::app);
        ofs << other << ",";
        ofs << this->trial << ",";
        ofs << this->do_excess(this->start_state, 0, 0) + this->error_ << ",";
        ofs << this->lower_bound_->getValueAt(this->start_state) << ",";
        ofs << this->upper_bound_->getValueAt(this->start_state) << ",";
        ofs << this->lower_bound_->getSize() << ",";
        ofs << this->upper_bound_->getSize() << ",";
        ofs << std::static_pointer_cast<OccupancyMDP>(this->world_)->getMDPGraph()->getNumNodes() << ",";
        number num_max_jhist = 0, tmp;
        for (const auto &state : std::static_pointer_cast<OccupancyMDP>(this->world_)->getStoredStates())
        {
            if (num_max_jhist < (tmp = state->toOccupancyState()->getJointHistories().size()))
            {
                num_max_jhist = tmp;
            }
        }
        ofs << num_max_jhist << ",";
        ofs << this->duration;

        ofs << "\n";
        ofs.close();
    }

    double HSVI::getResult()
    {
        return this->lower_bound_->getValueAt(this->world_->getInitialState());
    }

// #ifdef LOGTIME
    void HSVI::StartTime()
    {
        this->time_start = std::chrono::high_resolution_clock::now();
    }

    void HSVI::updateTime(std::string information)
    {
        if (information == "Action")
        {
            HSVI::TIME_IN_SELECT_ACTION += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->time_start).count();
        }
        else if (information == "Update Lower")
        {
            HSVI::TIME_IN_UPDATE_LB += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->time_start).count();
        }
        else if (information == "Update Upper")
        {
            HSVI::TIME_IN_UPDATE_UB += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->time_start).count();
        }
        else if (information == "Next State")
        {
            HSVI::TIME_IN_SELECT_STATE += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->time_start).count();
        }
        else if (information == "Intialisation")
        {
            HSVI::TIME_IN_SELECT_STATE += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->time_start).count();
        }
    }

    void HSVI::printTime()
    {
        // std::cout << "Total Time Select Action :" << this->total_time_select_action << std::endl;
        // std::cout << "Total Time Update Lower :" << this->total_time_update_lower << std::endl;
        // std::cout << "Total Time Update Upper :" << this->total_time_update_upper << std::endl;
        // std::cout << "Total Time Determien Next State : " << this->total_time_next_state << std::endl;

        // std::cout << "\n Total Time Lower Bound : " << std::endl;
        // this->lower_bound_->printTime();

        // std::cout << "\n Total Time Upper Bound : " << std::endl;
        // this->upper_bound_->printTime();
    }
// #endif

} // namespace sdm