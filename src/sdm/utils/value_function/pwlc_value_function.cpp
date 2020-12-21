#include <sdm/utils/pwlc_value_function.hpp>

namespace sdm
{

    PWLCValueFunction::PWLCValueFunction(number horizon)
    {
    }

    LowerBound PWLCValueFunction::getLowerBound()
    {
        return this->lower_bound;
    }

    double PWLCValueFunction::getValueAt(TState state) const
    {
        return this->lower_bound.getValueAt(state);
    }

    double PWLCValueFunction::getGreedyValueAt(TState &belief, TAction &dr, std::shared_ptr<Vector> &value) const
    {
    }

    void PWLCValueFunction::prune()
    {
    }

    void PWLCValueFunction::classic_pruning()
    {
    }
    void PWLCValueFunction::bounded_pruning()
    {
    }
    void PWLCValueFunction::lark_pruning()
    {
    }

    std::shared_ptr<Vector> PWLCValueFunction::getMinimumAlphaVector(const std::shared_ptr<Vector> &, const std::shared_ptr<Vector> &)
    {
    }

    std::pair<std::shared_ptr<Vector>, double> PWLCValueFunction::checkDominance(const std::shared_ptr<Vector> &, const std::unordered_set<std::shared_ptr<Vector>> &)
    {
    }

    void PWLCValueFunction::initialize()
    {

        // double value;

        // this->container.emplace(this->planning_horizon, std::unordered_map<std::shared_ptr<Vector>, std::shared_ptr<Vector>>());
        // this->mdp_vf.emplace(this->planning_horizon, std::make_shared<Vector>(common::model->getNumStates()));
        // this->mdp_vf.at(this->planning_horizon)->init(0.0);

        // std::cout << "_coucou1_1" << std::endl;
        for (auto h = this->planning_horizon - 1; h >= 0; --h)
        {
            // upper bound initialization
            // this->upper_bound

            //     this->container.emplace(h, std::unordered_map<std::shared_ptr<Vector>, std::shared_ptr<Vector>>());
            // this->mdp_vf.emplace(h, std::make_shared<Vector>(common::model->getNumStates()));
            // this->mdp_vf.at(h)->init(std::numeric_limits<double>::min());
            // for (state x = 0; x < common::model->getNumStates(); ++x)
            // {
            //     for (action u = 0; u < common::model->getNumActions(); ++u)
            //     {
            //         value = this->world.getReward(x, u);

            //         for (state y = 0; y < common::model->getNumStates(); ++y)
            //         {
            //             value += common::model->getDiscount() * common::model->getTransitionProbability(x, u, y) * (*this->mdp_vf.at(h + 1))[y];
            //         }

            //         // std::cout << "_coucou1_4" << std::endl;
            //         (*this->mdp_vf.at(h))[x] = std::max(value, (*this->mdp_vf.at(h))[x]);
            //     }
            // }

            // lower bound initialization
            this->lower_bound.push_back(LowerBound(this->world->getNumStates()));
            this->lower_bound[h].setValueAt(this->world->getStartDistrib(), this->world->getMinReward() * (this->world->getPlanningHorizon() - h));
        }
    }

    void PWLCValueFunction::updateValueAt(TState &state)
    {
        this->lower_bound.updateValueAt(state, this->backup(state));
    }
} // namespace sdm