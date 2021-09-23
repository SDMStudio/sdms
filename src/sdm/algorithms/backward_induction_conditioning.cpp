#include <sdm/algorithms/backward_induction.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

namespace sdm
{
    BackwardInduction::BackwardInduction(std::shared_ptr<SolvableByHSVI> &world,
               number planning_horizon,
               std::string name) : world_(world),
                                  planning_horizon_(planning_horizon),
                                  name_(name)
    {
        auto tabular_backup = std::make_shared<TabularBackup>(world);
        auto action_tabular = std::make_shared<ActionVFTabulaire>(world);

        auto init= std::make_shared<MinInitializer>(world);

        this->bound_ = std::make_shared<TabularValueFunction>(planning_horizon, init, tabular_backup, action_tabular, true);
    }

    std::shared_ptr<BackwardInduction> BackwardInduction::getptr()
    {
        return this->shared_from_this();
    }

    // void BackwardInduction::initLogger()
    // {

    //     // ************* Global Logger ****************
    //     std::string format = "#> Trial :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\t Size_lower_bound({}) \t Size_upper_bound({}) \t Time({})  \n";

    //     // Build a logger that prints logs on the standard output stream
    //     auto std_logger = std::make_shared<sdm::StdLogger>(format);

    //     // Build a logger that prints logs in a file
    //     // auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);

    //     // Build a logger that stores data in a CSV file
    //     auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Size_lower_bound", "Size_upper_bound", "Time"});

    //     // Build a multi logger that combines previous loggers
    //     this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
        
    //     // ************* Precise Logger ****************
    //     format = "#> Trial :\t{}Horizon :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\t Size_lb({}) \t Size_ub({}) \t Action_Time({}) \t State_Time({}) \t Update_LB_Time({}) \t Update_UB_Time({}) \t Size_JHistories({}) \t Size_IHistory({}) \t Size_ActionSpace ({})  \n";

    //     // Build a logger that prints logs on the standard output stream
    //     std_logger = std::make_shared<sdm::StdLogger>(format);

    //     // Build a logger that stores data in a CSV file
    //     csv_logger = std::make_shared<sdm::CSVLogger>(this->name_+"_precise_data", std::vector<std::string>{"Trial","Horizon","Error", "Value_LB", "Value_UB", "Size_lb", "Size_ub", "Action_Time","State_Time","Update_LB_Time","Update_UB_Time","Size_JHistories","Size_IHistory","Size_ActionSpace"});

    //     // Build a multi logger that combines previous loggers
    //     this->logger_precise_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{csv_logger});
    // }

    void BackwardInduction::do_initialize()
    {
        this->bound_->initialize();
    }

    void BackwardInduction::do_solve()
    {
        std::cout << "\n\n###############################################################\n";
        std::cout << "#############    Start BackwardInduction \"" << this->name_ << "\"    ####################\n";
        std::cout << "###############################################################\n\n";

        this->start_state = this->world_->getInitialState();

        // this->start_time = std::chrono::high_resolution_clock::now();

        do
        {
            // Logging (save data and print algorithms variables)
            //---------------------------------//
            // this->current_time = std::chrono::high_resolution_clock::now();
            // this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration- BackwardInduction::TIME_TO_REMOVE);
            // this->updateTime(current_time,"Time_to_remove");

            //---------------------------------//

            this->do_explore(start_state, 0, 0);

            // this->duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - this->start_time).count();

        } while (false);

        std::cout << "#>Value Final, s h:" << 0 << "\t V_(" << this->bound_->getValueAt(start_state, 0) << ")"<< std::endl;
        //---------------------------------//
        // this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), this->lower_bound_->getSize(), this->upper_bound_->getSize(), this->duration- BackwardInduction::TIME_TO_REMOVE);
        //---------------------------------//
    }

    bool BackwardInduction::do_stop(const std::shared_ptr<State> &, double , number h)
    {
        return this->planning_horizon_<= h;
    }

    void BackwardInduction::do_explore(const std::shared_ptr<State> &state, double cost_so_far, number h)
    {
        //ON veut : 

        //For All A0:t 
        //  For All Indiv History 
        //      For All Indiv Action 
        //          Compute Q ;
        //  Take best Q ; 


        try
        {
            if (!this->do_stop(state, cost_so_far, h))
            {
                // std::cout << "#>Forward, s h:" << h << "\t V_(" << this->bound_->getValueAt(state, h) << ")"<< std::endl;

                auto action_space = this->world_->getActionSpaceAt(state, h);

                double best_value = -std::numeric_limits<double>::max();
                double resultat_backpropagation;
                for (const auto& action : *action_space)
                {
                    resultat_backpropagation = this->world_->getReward(state,action->toAction(),h);

                    auto observation_space = this->world_->getObservationSpaceAt(state,action->toAction(),h);

                    for(const auto& observation : *observation_space)
                    {
                        auto [next_state,proba] = this->world_->getNextState(this->bound_,state,action->toAction(),observation->toObservation(),h);
                        this->do_explore(next_state,cost_so_far + this->world_->getDiscount(h) * this->world_->getReward(state, action->toAction(), h),h+1);

                        resultat_backpropagation += this->world_->getDiscount(h) * proba * this->bound_->getValueAt(next_state,h+1);
                    }

                    if (best_value < resultat_backpropagation)
                    {
                        best_value = resultat_backpropagation;
                    }
                }
                this->bound_->updateValueAt(state,h,best_value);
            }
            //---------------DEBUG-----------------//
            if(h == 1)
            {
                std::cout << "#>Backward, s h:" << h << "\t V_(" << this->bound_->getValueAt(state, h) << ")"<< std::endl;
            }
            //-----------------DEBUG----------------//

            // ------------- TEST ------------
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "BackwardInduction::do_explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    double BackwardInduction::do_excess(const std::shared_ptr<State> &, double, number )
    {
        // try
        // {
        //     const auto &lb = this->lower_bound_->getValueAt(s, h);
        //     const auto &ub = this->upper_bound_->getValueAt(s, h);
        //     const auto &incumbent = this->lower_bound_->getValueAt(this->world_->getInitialState());
        //     double value_do_excess = this->world_->do_excess(incumbent, lb, ub, cost_so_far, this->error_, h);

        //     this->updateTime(current_time, "Do_excess");
        //     return value_do_excess;
        // }
        // catch (const std::exception &exc)
        // {
        //     // catch anything thrown within try block that derives from std::exception
        //     std::cerr << "BackwardInduction::do_excess(..) exception caught: " << exc.what() << std::endl;
        //     exit(-1);
        // }
    }

    void BackwardInduction::do_test()
    {
        // std::shared_ptr<State> ostate = this->world_->getInitialState();
        // std::shared_ptr<Action> jdr;
        // number end = (this->planning_horizon_ > 0) ? this->planning_horizon_ : 10;
        // for (number i = 0; i < end; i++)
        // {
        //     std::cout << "\n------------------------\nTIMESTEP " << i << "\n------------------------\n"
        //               << std::endl;
        //     jdr = this->lower_bound_->getBestAction(ostate, i);
        //     std::cout << "#> State\n"
        //               << ostate->str() << "\n"
        //               << std::endl;
        //     std::cout << "#> Action\n"
        //               << jdr->str() << std::endl;
        //     ostate = this->world_->nextState(ostate, jdr, i, this->getptr());
        // }
    }

    void BackwardInduction::do_save()
    {
        // this->getLowerBound()->save(this->name_ + "_lb");
    }

    std::shared_ptr<ValueFunction> BackwardInduction::getBound() const
    {
        return this->bound_;
    }

    void BackwardInduction::saveResults(std::string , double )
    {
        // std::ofstream ofs;
        // ofs.open(filename+ "hsvi_profiling.md", std::ios::out | std::ios::app);
        // ofs << "## " << filename << std::endl;
        // ofs << "| Trials \t"<< this->trial << std::endl;
        // ofs <<"| Error \t"<< this->do_excess(this->start_state, 0, 0) + this->error_ << std::endl;
        // ofs <<"| Time \t "<<this->duration<<std::endl;
        // ofs <<"| Lower Bound Value \t"<< this->lower_bound_->getValueAt(this->start_state) << std::endl;
        // ofs <<"| Upper Bound Value \t"<<this->upper_bound_->getValueAt(this->start_state) << std::endl;
        // ofs <<"| Total Size Lower Bound \t"<< this->lower_bound_->getSize() << std::endl;
        // ofs <<"| Total Size Upper Bound \t"<< this->upper_bound_->getSize() << std::endl;

        // ofs <<"| Horizon \t \t \t| Size Lower Bound \t \t \t| Size Upper Bound \t \t \t "<< std::endl;
        // for (size_t i = 0; i < this->planning_horizon_; i++)
        // {
        //     ofs<<"| Horizon \t"<<i<<"|"<<this->lower_bound_->getSize(i)<<"|"<<this->upper_bound_->getSize(i)<< std::endl;
        // }
        
        // ofs <<"| Number of Node \t"<< std::static_pointer_cast<OccupancyMDP>(this->world_)->getMDPGraph()->getNumNodes() << std::endl;
        // number num_max_jhist = 0, tmp;
        // for (const auto &state : std::static_pointer_cast<OccupancyMDP>(this->world_)->getStoredStates())
        // {
        //     if (num_max_jhist < (tmp = state->toOccupancyState()->getJointHistories().size()))
        //     {
        //         num_max_jhist = tmp;
        //     }
        // }
        // ofs <<"| Max number of JHistory \t "<< num_max_jhist << std::endl;
        // ofs.close();
    }

    double BackwardInduction::getResult()
    {
        return this->bound_->getValueAt(this->world_->getInitialState());
    }

    // void BackwardInduction::updateTime(std::chrono::high_resolution_clock::time_point start_time, std::string information)
    // {
    //     auto time = std::Performance::computeTime(start_time);
        
    //     if (information == "Action")
    //     {
    //         BackwardInduction::TIME_IN_SELECT_ACTION += time;
    //     }
    //     else if (information == "Update Lower")
    //     {
    //         BackwardInduction::TIME_IN_UPDATE_LB += time;
    //     }
    //     else if (information == "Update Upper")
    //     {
    //         BackwardInduction::TIME_IN_UPDATE_UB += time;
    //     }
    //     else if (information == "Next State")
    //     {
    //         BackwardInduction::TIME_IN_SELECT_STATE += time;
    //     }
    //     else if (information == "Intialisation")
    //     {
    //         // BackwardInduction::TIME_IN_SELECT_STATE += time;
    //     }else if(information == "Time_to_remove")
    //     {
    //         BackwardInduction::TIME_TO_REMOVE += time;
    //     }
    //     else if(information == "Pruning LB")
    //     {
    //         BackwardInduction::TIME_IN_PRUNING_LB += time;
    //     }else if(information == "Pruning UB")
    //     {
    //         BackwardInduction::TIME_IN_PRUNING_UB += time;
    //     }
    //     else if(information == "Do_excess")
    //     {
    //         BackwardInduction::TIME_IN_DO_EXCESS += time;
    //     }
    // }

    // void BackwardInduction::cleanTIME()
    // {
    //     BackwardInduction::TIME_IN_SELECT_STATE = 0;
    //     BackwardInduction::TIME_IN_SELECT_ACTION = 0;
    //     BackwardInduction::TIME_INITIALIZATION = 0;
    //     BackwardInduction::TIME_IN_UPDATE_LB = 0;
    //     BackwardInduction::TIME_IN_UPDATE_UB = 0;
    //     BackwardInduction::TIME_IN_PRUNING_LB = 0;
    //     BackwardInduction::TIME_IN_PRUNING_UB = 0;
    //     BackwardInduction::TIME_IN_DO_EXCESS =0;
    //     BackwardInduction::TIME_TO_REMOVE =0;
    // }
    
} // namespace sdm