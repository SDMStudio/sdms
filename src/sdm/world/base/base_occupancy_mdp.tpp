#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    BaseOccupancyMDP<TState, TAction>::BaseOccupancyMDP()
    {
    }

    template <typename TState, typename TAction>
    BaseOccupancyMDP<TState, TAction>::BaseOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : dpomdp_(underlying_dpomdp)
    {
        typename TState::jhistory_type jhist;
        jhist = std::make_shared<typename TState::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), (hist_length > 0) ? hist_length : -1);

        this->initial_history_ = jhist;
        this->initial_state_ = std::make_shared<TState>();
        this->current_state_ = std::make_shared<TState>();

        for (typename TState::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                auto p_x_h = std::make_pair(s, jhist);
                this->initial_state_->setProbabilityAt(p_x_h, this->dpomdp_->getStartDistrib().probabilities()[s]);
            }
        }
        this->initial_state_->finalize();
        this->initial_state_->setFullyUncompressedOccupancy(this->initial_state_->getptr());
        this->initial_state_->setOneStepUncompressedOccupancy(this->initial_state_->getptr());
    }

    template <typename TState, typename TAction>
    BaseOccupancyMDP<TState, TAction>::BaseOccupancyMDP(std::string underlying_dpomdp, number hist_length) : BaseOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename TState, typename TAction>
    TState &BaseOccupancyMDP<TState, TAction>::getState()
    {
        return *this->current_state_;
    }

    template <typename TState, typename TAction>
    TState BaseOccupancyMDP<TState, TAction>::reset()
    {
        // Reset the joint history to initial value
        this->current_history_ = this->initial_history_;

        // Reset the occupancy state
        *this->current_state_ = *this->initial_state_;
        
        // Reset the underlying DecPOMDP
        this->dpomdp_->reset();
        
        // Return the occupancy (which is the observation in BaseOccupancyMDP formalism)
        return *this->current_state_;
    }

    template <typename TState, typename TAction>
    std::tuple<TState, std::vector<double>, bool> BaseOccupancyMDP<TState, TAction>::step(TAction joint_idr)
    {
        // Select joint action 
        auto jaction = joint_idr.act(this->current_state_->getJointLabels(this->current_history_->getIndividualHistories())); 
        
        // Do a step on the DecPOMDP and get next observation and rewards
        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);
        
        // Expand the current history
        this->current_history_ = this->current_history_->expand(next_obs);
        
        // Compute the next compressed occupancy state
        *this->current_state_ = this->nextState(*this->current_state_, joint_idr); 

        // return the new occupancy state and the perceived rewards
        return std::make_tuple(*this->current_state_, rewards, done); 
    }

    template <typename TState, typename TAction>
    bool BaseOccupancyMDP<TState, TAction>::isSerialized() const
    {
        return false;
    }

    template <typename TState, typename TAction>
    DiscreteDecPOMDP *BaseOccupancyMDP<TState, TAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename TState, typename TAction>
    TState BaseOccupancyMDP<TState, TAction>::getInitialState()
    {
        return *this->initial_state_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> BaseOccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        using decision_rule_t = typename TAction::value_type;

        // Get possible histories for all agents
        auto vect_i_hist = ostate.getAllIndividualHistories();

        // Get individual decision rules for each agent
        std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
        for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            auto vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
            FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());
            vect_i_dr.push_back(f_indiv_dr_space.getAll());
        }

        // Get joint decision rules for each agent
        std::vector<TAction> vect_j_dr = {};
        for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
        {
            vect_j_dr.push_back(TAction(joint_idr));
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<TAction>>(vect_j_dr);
    }

    template <typename TState, typename TAction>
    TState BaseOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number, std::shared_ptr<HSVI<TState, TAction>>, bool compression) const
    {
        // The new compressed occupancy state
        std::shared_ptr<TState> new_compressed_occupancy_state;
        // The new fully uncompressed occupancy state
        std::shared_ptr<TState> new_fully_uncompressed_occupancy_state = std::make_shared<TState>();
        // The new one step left occupancy state
        std::shared_ptr<TState> new_one_step_left_compressed_occupancy_state = std::make_shared<TState>();

        // for all element in the support of the fully uncompressed occupancy state
        for (auto &p_x_o : *ostate.getFullyUncompressedOccupancy())
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;

            // Get joint action based on a joint decision rule and a joint labels
            auto jaction = joint_idr.act(ostate.getJointLabels(o->getIndividualHistories()));

            for (auto &y : this->dpomdp_->getReachableStates(x, jaction))
            {
                for (auto &z : this->dpomdp_->getReachableObservations(x, jaction, y))
                {
                    // Get the probability of the next couple (next_state, next_joint history)
                    double next_occupancy_measure = p_x_o.second * this->dpomdp_->getObsDynamics()->getDynamics(x, this->dpomdp_->getActionSpace()->joint2single(jaction), this->dpomdp_->getObsSpace()->joint2single(z), y);

                    // If occupancy measure is greater than zero, we build our occupancy states
                    if (next_occupancy_measure > 0)
                    {
                        // Build fully uncompressed occupancy state
                        auto joint_history_next = o->expand(z);
                        new_fully_uncompressed_occupancy_state->addProbabilityAt({y, joint_history_next}, next_occupancy_measure);

                        // Build one step left uncompressed occupancy state
                        auto compressed_joint_history = ostate.getCompressedJointHistory(o);
                        auto compressed_joint_history_next = compressed_joint_history->expand(z);
                        new_one_step_left_compressed_occupancy_state->addProbabilityAt({y, compressed_joint_history_next}, next_occupancy_measure);

                        // Update next history labels
                        new_one_step_left_compressed_occupancy_state->updateJointLabels(joint_history_next->getIndividualHistories(), compressed_joint_history_next->getIndividualHistories());
                    }
                }
            }
        }

        // Finalize the one step left compressed occupancy state
        new_one_step_left_compressed_occupancy_state->finalize();

        if (compression)
        {
            // Compress the occupancy state
            new_compressed_occupancy_state = std::make_shared<TState>(new_one_step_left_compressed_occupancy_state->compress());
            new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
            new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
            return *new_compressed_occupancy_state;
        }

        new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state->getptr());
        new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state->getptr());
        return *new_one_step_left_compressed_occupancy_state;
    }

    template <typename TState, typename TAction>
    TState BaseOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &joint_idr, number h, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        return this->nextState(ostate, joint_idr, h, hsvi, true);
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::getReward(const TState &ostate, const TAction &joint_idr) const
    {
        double r = 0;
        for (auto &p_x_o : ostate)
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;
            auto jaction = joint_idr.act(o->getIndividualHistories());
            r += p_x_o.second * this->dpomdp_->getReward()->getReward(x, this->dpomdp_->getActionSpace()->joint2single(jaction));

            // REPLACE BY IN LATER VERSION
            // r += p_x_o.second * this->dpomdp_->getReward(x, jaction);
        }
        return r;
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const TState &ostate, const TAction &oaction, number t) const
    {
        TState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteMDP> BaseOccupancyMDP<TState, TAction>::toMDP()
    {
        return this->dpomdp_->toMDP();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> BaseOccupancyMDP<TState, TAction>::toBeliefMDP()
    {
        return this->dpomdp_->toBeliefMDP();
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::getDiscount(number)
    {
        return this->dpomdp_->getDiscount();
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::getWeightedDiscount(number horizon)
    {
        return std::pow(this->dpomdp_->getDiscount(), horizon);
    }

    template <typename TState, typename TAction>
    double BaseOccupancyMDP<TState, TAction>::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon)
    {
        return std::min(ub - lb, cost_so_far + this->dpomdp_->getDiscount() * ub - incumbent) - error / this->getWeightedDiscount(horizon);
    }

    template <typename TState, typename TAction>
    TAction BaseOccupancyMDP<TState, TAction>::selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>> &, const std::shared_ptr<ValueFunction<TState, TAction>> &ub, const TState &s, number h)
    {
        return ub->getBestAction(s, h);
    }

} // namespace sdm
