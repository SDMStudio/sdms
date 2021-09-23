#include <sdm/algorithms/backward_induction.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

namespace sdm
{
    BackwardInduction::BackwardInduction(std::shared_ptr<SolvableByHSVI> &world,
               std::string name) : world_(world),
                                  name_(name)
    {
        auto tabular_backup = std::make_shared<TabularBackup>(world);
        auto action_tabular = std::make_shared<ActionVFTabulaire>(world);

        auto init= std::make_shared<MinInitializer>(world);

        this->bound_ = std::make_shared<TabularValueFunction>(this->world_->getUnderlyingProblem()->getHorizon(), init, tabular_backup, action_tabular, true);
    }

    std::shared_ptr<BackwardInduction> BackwardInduction::getptr()
    {
        return this->shared_from_this();
    }

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

        this->do_explore(start_state, 0, 0);

        std::cout << "#>Value Final, s h:" << 0 << "\t V_(" << this->bound_->getValueAt(start_state, 0) << ")"<< std::endl;
    }

    bool BackwardInduction::do_stop(const std::shared_ptr<State> &, double , number h)
    {
        return this->world_->getUnderlyingProblem()->getHorizon()<= h;
    }

    void BackwardInduction::do_explore(const std::shared_ptr<State> &state, double cost_so_far, number h)
    {
        try
        {
            if (!this->do_stop(state, cost_so_far, h))
            {
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

    void BackwardInduction::do_test()
    {
    }

    void BackwardInduction::do_save()
    {
    }

    std::shared_ptr<ValueFunction> BackwardInduction::getBound() const
    {
        return this->bound_;
    }

    void BackwardInduction::saveResults(std::string , double )
    {
    }

    double BackwardInduction::getResult()
    {
        return this->bound_->getValueAt(this->world_->getInitialState());
    }
    
} // namespace sdm