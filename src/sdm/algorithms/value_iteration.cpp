
#include <sdm/algorithms/value_iteration.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
// #include <sdm/world/belief_mdp.hpp>

namespace sdm
{
    ValueIteration::ValueIteration(std::shared_ptr<SolvableByHSVI> problem, double error, int horizon) : problem_(problem), error_(error), horizon_(horizon)
    {
    }

    /**
     * @brief Initialize the algorithm
     */
    void ValueIteration::do_initialize()
    {
        auto under_pb = this->problem_->getUnderlyingProblem();

        auto tabular_backup = std::make_shared<TabularBackup>(this->problem_);
        auto action_tabular = std::make_shared<ActionVFTabulaire>(this->problem_);

        auto init_ub = std::make_shared<MaxInitializer>(this->problem_);

        this->policy_evaluation_1_ = std::make_shared<TabularValueFunction>(under_pb->getHorizon(), init_ub, tabular_backup, action_tabular, false);
        this->policy_evaluation_2_ = std::make_shared<TabularValueFunction>(under_pb->getHorizon(), init_ub, tabular_backup, action_tabular, false);

        policy_evaluation_1_->initialize();
        policy_evaluation_2_->initialize();
    }

    /**
     * @brief Solve a problem solvable by HSVI. 
     */
    void ValueIteration::do_solve()
    {
        auto under_pb = this->problem_->getUnderlyingProblem();
        double max_value;
        do
        {
            this->policy_evaluation_1_ = std::make_shared<TabularValueFunction>(*this->policy_evaluation_2_);

            max_value = -std::numeric_limits<double>::max();

            for (int t = this->horizon_ - 1; t >= 0; t--)
            {
                for (const auto &state : *under_pb->getStateSpace(t))
                {
                    this->policy_evaluation_2_->updateValueAt(state->toState(), t);
                    max_value = std::max(std::abs(policy_evaluation_1_->getValueAt(state->toState(), t) - policy_evaluation_2_->getValueAt(state->toState(), t)), max_value);
                }
            }
        } while (max_value > this->error_);
    }

    void ValueIteration::determinedAllNextState()
    {
        for (size_t i = 0; i < this->horizon_; i++)
        {
            this->all_state.push_back(std::vector<std::shared_ptr<State>>());
        }

        auto initial_state = this->problem_->getInitialState();
        this->all_state[0].push_back(initial_state);

        this->determinedAllNextStateRecursive(initial_state, 0);
    }

    void ValueIteration::determinedAllNextStateRecursive(const std::shared_ptr<State> &, number)
    {
        // for (const auto& it = this->problem_->getActionSpaceAt(state, t)->begin() ; it != this->problem_->getActionSpaceAt(state, t)->end();++it)
        // {
        //     auto accessible_observation_space = std::static_pointer_cast<BeliefMDP>(this->problem_)->getObservationSpace(t);
        //     for (const auto &observation : *accessible_observation_space)
        //     {
        //         bool skip_compute_next_state = (value_function->isFiniteHorizon() && ((t + 1) >= value_function->getHorizon()));
        //         // Compute next state (if required)
        //         // std::cout<<"Calcul ?"<<std::endl;
        //         auto [next_state, state_transition_proba] = (skip_compute_next_state) ? Pair<std::shared_ptr<State>, double>({nullptr, 1.}) : this->nextBeliefAndProba(belief, action, observation->toObservation(), t);
        //     }
        // }
    }

    /**
     * @brief Test the learnt value function on one episode
     */
    void ValueIteration::do_test()
    {
    }

    bool ValueIteration::borne()
    {
        double max_value = -std::numeric_limits<double>::max();
        for (auto &state : *this->problem_->getUnderlyingProblem()->getStateSpace(0))
        {
            max_value = std::max(std::abs(policy_evaluation_1_->getValueAt(state->toState(), 0) - policy_evaluation_2_->getValueAt(state->toState(), 0)), max_value);
        }
        std::cout << "\n Error : " << max_value << std::endl;

        if (max_value <= this->error_)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    std::shared_ptr<ValueFunction> ValueIteration::getValueFunction()
    {
        return this->policy_evaluation_2_;
    }

    double ValueIteration::getResult()
    {
        return this->policy_evaluation_2_->getValueAt(this->problem_->getInitialState());
    }
}
