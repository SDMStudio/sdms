#include <sdm/exception.hpp>

#include <sdm/algorithms/hsvi.hpp>

namespace sdm
{
    HSVI::HSVI(std::shared_ptr<SolvableByHSVI> &world,
               std::shared_ptr<ValueFunction> lower_bound,
               std::shared_ptr<ValueFunction> upper_bound,
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

    std::shared_ptr<HSVI> HSVI::getptr()
    {
        return this->shared_from_this();
    }

    void HSVI::initLogger()
    {
        std::string format = "#> Trial :\t{}\tError :\t{}\t->\tV_lb({})\tV_ub({})\n";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that prints logs in a file
        auto file_logger = std::make_shared<sdm::FileLogger>(this->name_ + ".txt", format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(this->name_, std::vector<std::string>{"Trial", "Error", "Value_LB", "Value_UB", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger_ = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, file_logger, csv_logger});
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

        std::shared_ptr<State> start_state = this->world_->getInitialState();

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
        } while (!this->do_stop(start_state, 0, 0));

        //---------------------------------//
        this->logger_->log(this->trial, this->do_excess(start_state, 0, 0) + this->error_, this->lower_bound_->getValueAt(start_state), this->upper_bound_->getValueAt(start_state), (float)(clock() - t_begin) / CLOCKS_PER_SEC);
        //std::cout << "Final LB : \n" << this->lower_bound_->str() << "Final UB : \n" << this->upper_bound_->str() << std::endl;
        //---------------------------------//
    }

    bool HSVI::do_stop(const std::shared_ptr<State> &s, double cost_so_far, number h)
    {
        return ((this->do_excess(s, cost_so_far, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    void HSVI::do_explore(const std::shared_ptr<State> &s, double cost_so_far, number h)
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
                std::shared_ptr<Action> a = this->world_->selectNextAction(this->lower_bound_, this->upper_bound_, s, h);
                std::shared_ptr<State> s_ = this->world_->nextState(s, a, h, this->getptr());

                // Recursive explore
                this->do_explore(s_, cost_so_far + this->world_->getDiscount(h) * this->world_->getReward(s, a, h), h + 1);

                // Update bounds
                this->lower_bound_->updateValueAt(s, h);

                this->upper_bound_->updateValueAt(s, h);

                std::cout << "Lower Bound\n" << *this->lower_bound_ << std::endl;
            }

            //---------------DEBUG-----------------//
            std::cout << "\t\t#> h:" << h << "\t V_lb(" << this->lower_bound_->getValueAt(s, h) << ")\tV_ub(" << this->upper_bound_->getValueAt(s, h) << ")" << std::endl;
            //-----------------DEBUG----------------//
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
        std::shared_ptr<State> ostate = this->world_->getInitialState();
        std::shared_ptr<Action> jdr;
        number end = (this->planning_horizon_ > 0) ? this->planning_horizon_ : 10;
        for (number i = 0; i < end; i++)
        {
            std::cout << "\n------------------------\nTIMESTEP " << i << "\n------------------------\n"
                      << std::endl;
            jdr = this->lower_bound_->getBestAction(ostate, i);
            std::cout << "#> State\n"
                      << ostate << "\n"
                      << std::endl;
            std::cout << "#> Action\n"
                      << jdr << std::endl;
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
} // namespace sdm