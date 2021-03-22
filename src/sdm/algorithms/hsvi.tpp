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
                                number num_max_trials) : world_(world),
                                                         lower_bound_(lower_bound),
                                                         upper_bound_(upper_bound),
                                                         error_(error),
                                                         planning_horizon_(planning_horizon)
    {
        this->MAX_TRIALS = num_max_trials;
        this->do_initialize();
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::initLogger()
    {
        std::string format = "#> Trial : {}\tError : {}\t\tV_lb({}) / V_ub({})\n";

        auto std_logger = std::make_shared<sdm::StdLogger>(format);
        auto file_logger = std::make_shared<sdm::FileLogger>("hsvi.txt", format);
        auto csv_logger = std::make_shared<sdm::CSVLogger>("hsvi", std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Time"});

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

        clock_t t_begin, t_end;
        t_begin = clock();
        do
        {
            this->logger_->log(this->trial, this->do_excess(start_state, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
            // std::cout << "Trial : " << this->trial << "\tError : " << this->do_excess(start_state, 0) + this->error_ << std::endl;
            // std::cout << "LB : " << this->lower_bound_->str() << "UB : " << this->upper_bound_->str() << std::endl;
            this->do_explore(start_state, 0);
            this->trial++;
        } while (!this->do_stop(start_state, 0));

        t_end = clock();
        float temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;

        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "Number trials : " << this->trial << "\tError : " << this->do_excess(start_state, 0) + this->error_ << std::endl;
        std::cout << "Final LB : \n"
                  << this->lower_bound_->str() << "Final UB : \n"
                  << this->upper_bound_->str() << std::endl;
        printf("\n\nTemps = %f s\n", temps);
    }

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::do_excess(const TState &s, number h)
    {
        return (this->upper_bound_->getValueAt(s, h) - this->lower_bound_->getValueAt(s, h)) - this->error_ / std::pow(this->world_->getDiscount(), h);
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
} // namespace sdm