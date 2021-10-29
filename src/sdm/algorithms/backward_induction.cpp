#include <sdm/algorithms/backward_induction.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>

namespace sdm
{
    BackwardInduction::BackwardInduction(const std::shared_ptr<SolvableByHSVI> &world, std::string name) : DynamicProgramming(world, 0, name)
    {
        auto init = std::make_shared<MinInitializer>(world);
        auto action_tabular = std::make_shared<ExhaustiveActionSelection>(world);
        this->bound_ = std::make_shared<TabularValueFunction>(world, init, action_tabular);
        this->bound_->setUpdateOperator(std::make_shared<TabularUpdate>(this->bound_));
    }

    std::shared_ptr<BackwardInduction> BackwardInduction::getptr()
    {
        return this->shared_from_this();
    }

    void BackwardInduction::initialize()
    {
        this->bound_->initialize();
    }

    void BackwardInduction::solve()
    {
        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "START PLANNING" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" <<config::NO_COLOR << std::endl;

        this->start_state = getWorld()->getInitialState();

        this->explore(start_state, 0, 0);

        std::cout << config::LOG_SDMS << "FINALE VALUE : " << this->bound_->getValueAt(start_state, 0) << std::endl;


        std::cout << config::SDMS_THEME_1 << "------------------------------------" << std::endl;
        std::cout << config::LOG_SDMS << "END PLANNING" << std::endl;
        std::cout << config::SDMS_THEME_1 << "------------------------------------" <<config::NO_COLOR << std::endl;
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
                    resultat_backpropagation = getWorld()->getReward(state, action->toAction(), h);

                    // Go over all observations
                    auto observation_space = getWorld()->getObservationSpaceAt(state, action->toAction(), h);
                    for (const auto &observation : *observation_space)
                    {
                        // Compute next state and proba
                        auto [next_state, proba] = getWorld()->getNextState(state, action->toAction(), observation->toObservation(), h);

                        // Explore next state
                        this->explore(next_state, cost_so_far + getWorld()->getDiscount(h) * getWorld()->getReward(state, action->toAction(), h), h + 1);

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
                std::cout << config::LOG_SDMS;
                for (int i = 0; i < h; i++)
                {
                    std::cout << "     ";
                }
                std::cout << "Horizon " << h << "    State " << state << "    Value " << this->bound_->getValueAt(state, h) << std::endl;
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

} // namespace sdm