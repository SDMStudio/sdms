#include <sdm/algorithms/backward_induction.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>

namespace sdm
{
    BackwardInduction::BackwardInduction(std::shared_ptr<SolvableByHSVI> &world, std::string name) : DynamicProgramming(world, 0, name)
    {
        // auto tabular_backup = std::make_shared<TabularBackup>(world);
        auto action_tabular = std::make_shared<ExhaustiveActionSelection>(world);

        auto init = std::make_shared<MinInitializer>(world);

        auto bound = std::make_shared<TabularValueFunction>(this->world_->getUnderlyingProblem()->getHorizon(), init, action_tabular);

        bound->setUpdateOperator(std::make_shared<TabularUpdate>(world, bound));

        this->bound_ = bound;

        
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
        std::cout << "\n\n###############################################################\n";
        std::cout << "#############    Start BackwardInduction \"" << this->name_ << "\"    ####################\n";
        std::cout << "###############################################################\n\n";

        this->start_state = this->world_->getInitialState();

        this->explore(start_state, 0, 0);

        std::cout << "#>Value Final, s h:" << 0 << "\t V_(" << this->bound_->getValueAt(start_state, 0) << ")" << std::endl;
    }

    bool BackwardInduction::stop(const std::shared_ptr<State> &, double, number h)
    {
        return this->world_->getUnderlyingProblem()->getHorizon() <= h;
    }

    void BackwardInduction::explore(const std::shared_ptr<State> &state, double cost_so_far, number h)
    {
        try
        {
            if (!this->stop(state, cost_so_far, h))
            {
                double best_value = -std::numeric_limits<double>::max(), resultat_backpropagation;

                // Go over all actions
                auto action_space = this->world_->getActionSpaceAt(state, h);
                for (const auto &action : *action_space)
                {
                    resultat_backpropagation = this->world_->getReward(state, action->toAction(), h);

                    // Go over all observations
                    auto observation_space = this->world_->getObservationSpaceAt(state, action->toAction(), h);
                    for (const auto &observation : *observation_space)
                    {
                        // Compute next state and proba
                        auto [next_state, proba] = this->world_->getNextState(this->bound_, state, action->toAction(), observation->toObservation(), h);
                        
                        // Explore next state
                        this->explore(next_state, cost_so_far + this->world_->getDiscount(h) * this->world_->getReward(state, action->toAction(), h), h + 1);

                        // Update backprop value
                        resultat_backpropagation += this->world_->getDiscount(h) * proba * this->bound_->getValueAt(next_state, h + 1);
                    }

                    if (best_value < resultat_backpropagation)
                    {
                        best_value = resultat_backpropagation;
                    }
                }
                this->bound_->setValueAt(state, best_value, h);
            }
            //---------------DEBUG-----------------//
            if (h == 1)
            {
                std::cout << "#>Backward, s h:" << h << "\t V_(" << this->bound_->getValueAt(state, h) << ")" << std::endl;
            }
            //-----------------DEBUG----------------//

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