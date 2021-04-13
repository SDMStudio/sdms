#include <sdm/algorithms/hsvi.hpp>

namespace sdm
{
    // template <typename TState, typename TAction>
    // HSVI<TState, TAction>::HSVI(number trials, std::string results)
    // {
    //     this->MAX_TRIALS = trials;
    //     // this->saveStatsBegins(results);
    // }

    template <typename TState, typename TAction>
    HSVI<TState, TAction>::HSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> &world,
                                std::shared_ptr<ValueFunction<TState, TAction>> lower_bound,
                                std::shared_ptr<ValueFunction<TState, TAction>> upper_bound,
                                number planning_horizon,
                                double error,
                                number num_max_trials,
                                std::string name) : world_(world),
                                                    lower_bound_(lower_bound),
                                                    upper_bound_(upper_bound),
                                                    error_(error),
                                                    planning_horizon_(planning_horizon),
                                                    name_(name)
    {
        this->MAX_TRIALS = num_max_trials;
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::initLogger()
    {
        std::string format = "#> Trial : {}\tError : {}\t\tV_lb({}) / V_ub({})\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Time"});

        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_initialize()
    {
        this->initLogger();

        this->lower_bound_->initialize();
        this->upper_bound_->initialize();
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_solve()
    {
        TState start_state = this->world_->getInitialState();
        this->trial = 0;
        // std::cout << *this->lower_bound_ << std::endl;

        clock_t t_begin = clock();
        do
        {
            // Logging (save data and print algorithms variables)
            this->logger_->log(this->trial, this->do_excess(start_state, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
            // std::cout << *this->lower_bound_ << std::endl;
            // std::cout << *this->upper_bound_ << std::endl;
            this->do_explore(start_state, 0);
            this->trial++;
        } while (!this->do_stop(start_state, 0));

        std::cout << "----------------------------------------------------" << std::endl;
        this->logger_->log(this->trial, this->do_excess(start_state, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
        std::cout << "Final LB : \n"
                  << this->lower_bound_->str() << "Final UB : \n"
                  << this->upper_bound_->str() << std::endl;
    }

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::do_excess(const TState &s, number h)
    {
        number realTime = h;
        if (this->world_->isSerialized())
        {
            // Compute the real time for serialized problem
            realTime = realTime / this->world_->getUnderlyingProblem()->getNumAgents();
        }
        return (this->upper_bound_->getValueAt(s, h) - this->lower_bound_->getValueAt(s, h) - this->error_) / std::pow(this->world_->getUnderlyingProblem()->getDiscount(), realTime);
    }

    template <typename TState, typename TAction>
    bool HSVI<TState, TAction>::do_stop(const TState &s, number h)
    {
        return ((this->do_excess(s, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_explore(const TState &s, number h)
    {
        if (!this->do_stop(s, h))
        {
            // Update bounds
            this->lower_bound_->updateValueAt(s, h);
            this->upper_bound_->updateValueAt(s, h);

            // Select next action and state following search process
            TAction a = this->selectNextAction(s, h);

            TState s_ = this->world_->nextState(s, a, h, this);

            // Recursive explore
            this->do_explore(s_, h + 1);

            // Update bounds
            this->lower_bound_->updateValueAt(s, h);
            this->upper_bound_->updateValueAt(s, h);
        }
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_test()
    {
        TState ostate = this->world_->getInitialState();
        TAction jdr;
        number end = (this->planning_horizon_ > 0) ? this->planning_horizon_ : 10;
        for (int i = 0; i < end; i++)
        {
            std::cout << "\n------------------------\nTIMESTEP " << i << "\n------------------------\n"
                      << std::endl;
            jdr = this->lower_bound_->getBestAction(ostate);
            std::cout << "#> State\n"
                      << ostate << "\n"
                      << std::endl;
            std::cout << "#> Action\n"
                      << jdr << std::endl;
            ostate = this->world_->nextState(ostate, jdr, i, this);
        }
    }

    template <typename TState, typename TAction>
    TAction HSVI<TState, TAction>::selectNextAction(const TState &s, number h)
    {
        return this->upper_bound_->getBestAction(s, h); // argmax_{a} q_value(s, a)
    }

    template <typename TState, typename TAction>
    std::shared_ptr<ValueFunction<TState, TAction>> HSVI<TState, TAction>::getLowerBound() const
    {
        return this->lower_bound_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<ValueFunction<TState, TAction>> HSVI<TState, TAction>::getUpperBound() const
    {
        return this->upper_bound_;
    }

    template <typename TState, typename TAction>
    int HSVI<TState, TAction>::getTrial() const
    {
        return this->trial;
    }

    // ****************************** Avec Gt ********************

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::do_excess_2(const TState &s, number h, double gt)
    {
        number realTime = h;

        if (this->world_->isSerialized())
        {
            // Compute the real time for serialized problem
            realTime = realTime / this->world_->getUnderlyingProblem()->getNumAgents();
        }
        return (this->upper_bound_->getValueAt(s, h) - this->lower_bound_->getValueAt(s, 0) - this->error_ + gt) / std::pow(this->world_->getUnderlyingProblem()->getDiscount(), realTime);
    }

    template <typename TState, typename TAction>
    bool HSVI<TState, TAction>::do_stop(const TState &s, number h, double gt)
    {
        return ((this->do_excess(s, h) <= 0) || (this->trial > this->MAX_TRIALS) || (this->do_excess_2(s, h, gt) <= 0));
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_explore(const TState &s, number h, double gt)
    {
        if (!this->do_stop(s, h, gt))
        {
            // Update bounds
            this->lower_bound_->updateValueAt(s, h);
            this->upper_bound_->updateValueAt(s, h);

            // Select next action and state following search process
            TAction a = this->selectNextAction(s, h);

            TState s_ = this->world_->nextState(s, a, h, this);

            // Recursive explore
            //this->do_explore(s_, h + 1);
            this->do_explore(s_, h + 1, gt + this->world_->getReward(s, a));

            // Update bounds
            this->lower_bound_->updateValueAt(s, h);
            this->upper_bound_->updateValueAt(s, h);
        }
    }

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::getResultOpti()
    {
        return this->lower_bound_->getValueAt(this->world_->getInitialState());
    }


} // namespace sdm