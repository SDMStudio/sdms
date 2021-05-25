#include <sdm/exception.hpp>

#include <sdm/algorithms/hsvi.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    HSVI<TState, TAction>::HSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> &world,
                                std::shared_ptr<ValueFunction<TState, TAction>> lower_bound,
                                std::shared_ptr<ValueFunction<TState, TAction>> upper_bound,
                                number planning_horizon,
                                double error,
                                number num_max_trials,
                                std::string name,
                                int time_max) : world_(world),
                                                    lower_bound_(lower_bound),
                                                    upper_bound_(upper_bound),
                                                    error_(error),
                                                    planning_horizon_(planning_horizon),
                                                    time_max_in_seconds_(time_max),
                                                    name_(name)
    {
        this->MAX_TRIALS = num_max_trials;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<HSVI<TState, TAction>> HSVI<TState, TAction>::getptr()
    {
        return this->shared_from_this();
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::initLogger()
    {
        std::string format = "#> Trial :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that prints logs in a file
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Time"});

        // Build a multi logger that combines previous loggers
        // this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{csv_logger});
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
        std::cout << "\n\n###############################################################\n";
        std::cout << "#############    Start HSVI \"" << this->name_ << "\"    ####################\n";
        std::cout << "###############################################################\n\n";

        const TState &start_state = this->world_->getInitialState();

        this->trial = 0;
        clock_t t_begin = clock();
        do
        {
            // Logging (save data and print algorithms variables)
            //---------------------------------//
            this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
            //---------------------------------//

            this->do_explore(start_state, 0, 0);
            this->trial++;
        } while (!this->do_stop(start_state, 0, 0) and time_max_in_seconds_ >= ((clock() - t_begin)/CLOCKS_PER_SEC));

        if(time_max_in_seconds_ < ((clock() - t_begin)/CLOCKS_PER_SEC))
        {
            std::cout<<"\n Time Limit "<<(clock() - t_begin)/CLOCKS_PER_SEC;
        }
        //---------------------------------//
        this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
        //std::cout << "Final LB : \n" << this->lower_bound_->str() << "Final UB : \n" << this->upper_bound_->str() << std::endl;
        //---------------------------------//
    }

    template <typename TState, typename TAction>
    bool HSVI<TState, TAction>::do_stop(const TState &s, double cost_so_far, number h)
    {
        return ((this->do_excess(s, cost_so_far, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_explore(const TState &s, double cost_so_far, number h)
    {
        try
        {
            if (!this->do_stop(s, cost_so_far, h))
            {
                if (this->lower_bound_->isInfiniteHorizon())
                {
                    this->lower_bound_->updateValueAt(s, h);
                    this->upper_bound_->updateValueAt(s, h);
                }

                // Select next action and state following search process
                const TAction &a = this->world_->selectNextAction(this->lower_bound_, this->upper_bound_, s, h);


                const TState &s_ = this->world_->nextState(s, a, h, this->getptr());

                // Recursive explore
                this->do_explore(s_, cost_so_far + this->world_->getDiscount(h) * this->world_->getReward(s, a), h + 1);

                // Update bounds
                this->lower_bound_->updateValueAt(s, h);
                this->upper_bound_->updateValueAt(s, h);
            }

            //---------------DEBUG-----------------//
            // std::cout << "\t\t#> h:" << h << "\t V_lb(" << this->lower_bound_->getValueAt(s, h) << ")\tV_ub(" << this->upper_bound_->getValueAt(s, h) << ")" << std::endl;
            //-----------------DEBUG----------------//
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "HSVI<TState, TAction>::do_explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::do_excess(const TState &s, double cost_so_far, number h)
    {
        try
        {
            const auto &lb = this->lower_bound_->getValueAt(s, h);
            const auto &ub = this->upper_bound_->getValueAt(s, h);
            const auto &incumbent = this->lower_bound_->getValueAt(this->world_->getInitialState());
            return this->world_->do_excess(incumbent, lb, ub, cost_so_far, this->error_, h);
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "HSVI<TState, TAction>::do_excess(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_test()
    {
        TState ostate = this->world_->getInitialState();
        TAction jdr;
        number end = (this->planning_horizon_ > 0) ? this->planning_horizon_ : 10;
        for (number i = 0; i < end; i++)
        {
            std::cout << "\n------------------------\nTIMESTEP " << i << "\n------------------------\n"
                      << std::endl;
            jdr = this->lower_bound_->getBestAction(ostate);
            std::cout << "#> State\n"
                      << ostate << "\n"
                      << std::endl;
            std::cout << "#> Action\n"
                      << jdr << std::endl;
            ostate = this->world_->nextState(ostate, jdr, i, this->getptr());
        }
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_save()
    {
        this->getLowerBound()->save(this->name_ + "_lb");
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
    int HSVI<TState, TAction>::getTrial()
    {
        return this->trial;
    }

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::getResultOpti()
    {
        return this->lower_bound_->getValueAt(this->world_->getInitialState());
    }

} // namespace sdm