#include <sdm/algorithms/planning/backward_induction.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>

namespace sdm
{
    BackwardInduction::BackwardInduction(const std::shared_ptr<SolvableByHSVI> &world, std::string name) : DynamicProgramming(world, 0, name)
    {
        auto init = std::make_shared<MinInitializer>(world);
        auto action_tabular = std::make_shared<ExhaustiveActionSelection>(world);
        this->bound_ = std::make_shared<TabularValueFunction>(world, init, action_tabular);
        this->bound_->setUpdateRule(std::make_shared<TabularUpdate>(this->bound_));
    }

    void BackwardInduction::initialize()
    {
        bound_->initialize();
        initLogger();
    }

    void BackwardInduction::solve()
    {
        printStartInfo();
        startExecutionTime();

        start_state = getWorld()->getInitialState();

        explore(start_state, 0, 0);

        std::cout << "\n" << config::LOG_SDMS << "FINALE VALUE : " << bound_->getValueAt(start_state, 0);

        printEndInfo();
    }

    bool BackwardInduction::stop(const std::shared_ptr<State> &, double, number h)
    {
        return (h >= this->getWorld()->getHorizon());
    }

    void BackwardInduction::explore(const std::shared_ptr<State> &state, double cost_so_far, number h)
    {
        try
        {
            if (!this->stop(state, cost_so_far, h))
            {
                double best_value = -std::numeric_limits<double>::max(), resultat_backpropagation;

                // Go over all actions
                auto action_space = getWorld()->getActionSpaceAt(state, h);
                for (const auto &action : *action_space)
                {
                    resultat_backpropagation = getWorld()->getReward(state, action, h);

                    // Go over all observations
                    auto observation_space = getWorld()->getObservationSpaceAt(state, action, h);
                    for (const auto &observation : *observation_space)
                    {
                        // Compute next state and proba
                        auto [next_state, proba] = getWorld()->getNextState(state, action, observation, h);

                        // Explore next state
                        this->explore(next_state, cost_so_far + getWorld()->getDiscount(h) * getWorld()->getReward(state, action, h), h + 1);

                        // Update backprop value
                        resultat_backpropagation += getWorld()->getDiscount(h) * proba * this->bound_->getValueAt(next_state, h + 1);
                    }

                    if (best_value < resultat_backpropagation)
                    {
                        best_value = resultat_backpropagation;
                    }
                }
                this->bound_->setValueAt(state, best_value, h);
            }
            if (h <= LOG_DEPTH)
            {
                // Print in loggers some execution variables
                logger->log(h, getBound()->getValueAt(start_state, 0), getBound()->getSize(), getExecutionTime());
            }

            // ------------- TEST ------------
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "BackwardInduction::explore(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void BackwardInduction::initLogger()
    {
        // ************* Global Logger ****************
        std::string format = "\r" + config::LOG_SDMS + "Horizon {:<8} Value {:<12.4f} Size {:<10} Time {:<12.4f}";

        // Build a logger that prints logs on the standard output stream
        auto std_logger = std::make_shared<sdm::StdLogger>(format);

        // Build a logger that stores data in a CSV file
        auto csv_logger = std::make_shared<sdm::CSVLogger>(name, std::vector<std::string>{"Horizon", "Value", "Size", "Time"});

        // Build a multi logger that combines previous loggers
        this->logger = std::make_shared<sdm::MultiLogger>(std::vector<std::shared_ptr<Logger>>{std_logger, csv_logger});
    }

    std::shared_ptr<BackwardInduction> BackwardInduction::getptr()
    {
        return this->shared_from_this();
    }

    void BackwardInduction::test()
    {
    }

    void BackwardInduction::save()
    {
    }

    std::shared_ptr<ValueFunction> BackwardInduction::getBound() const
    {
        return this->bound_;
    }

    std::string BackwardInduction::getAlgorithmName()
    {
        return "BackwardInduction";
    }
} // namespace sdm