#include <sdm/world/serialized_belief_mdp.hpp>

namespace sdm
{

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP()
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP(std::shared_ptr<SerializedMPOMDP> underlying_serial_mpomdp) : serialized_mpomdp_(underlying_serial_mpomdp)
    {
        // Creation of the initial state
        double proba = 0;

        // Go over all state at time step 0
        for (const auto &s : this->serialized_mpomdp_->getStateSpace(0)->getAll())
        {
            proba = this->serialized_mpomdp_->getStartDistrib().probabilities()[s.getState()];
            if (proba > 0)
            {
                this->initial_state_[s] = proba;
            }
        }
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp) : SerializedBeliefMDP(std::make_shared<SerializedMPOMDP>(underlying_dpomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP(std::string underlying_dpomdp) : SerializedBeliefMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    bool SerializedBeliefMDP<TBelief, TAction, TObservation>::isSerialized() const
    {
        return true;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedMPOMDP *SerializedBeliefMDP<TBelief, TAction, TObservation>::getUnderlyingProblem()
    {
        return this->serialized_mpomdp_.get();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::getInitialState()
    {
        return this->initial_state_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextStateSerialLastAgent(const TBelief &belief, const TAction &action, const TObservation &obs) const
    {

        TBelief new_belief;

        double tmp, obs_proba;
        // Go over all next serial state
        for (const auto &next_serial_state : this->serialized_mpomdp_->getStateSpace()->getAll())
        {
            tmp = 0;
            obs_proba = 0;
            // GO over all belief state
            for (const auto &belief_state : belief)
            {
                auto serial_state = belief_state.first;

                //Determine Transition probability and Observation Probability
                tmp += this->serialized_mpomdp_->getTransitionProbability(serial_state, action, next_serial_state) * belief_state.second;
                obs_proba += this->serialized_mpomdp_->getObservationProbability(serial_state, action, obs, next_serial_state);
            }

            // If observation probability and transition probability is not zero, we add a probability in the belief for the next serial state
            if (obs_proba && tmp)
            {
                new_belief[next_serial_state] = obs_proba * tmp;
            }
        }

        // Normalize the belief
        double sum = new_belief.norm_1();
        for (const auto &next_serial_state : new_belief)
        {
            new_belief[next_serial_state.first] = next_serial_state.second / sum;
        }
        return new_belief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextStateSerialStep(const TBelief &belief, const TAction &action) const
    {

        TBelief new_belief;
        new_belief.setAgent(belief.getCurrentAgentId() + 1);

        for (const auto &belief_state : belief)
        {
            auto serial_state = belief_state.first;

            // Update list of actions
            auto actions_list = serial_state.getAction();
            // Add the new action
            actions_list.push_back(action);

            // Create new serial state
            typename TBelief::state_type new_serialized_state(serial_state.first, actions_list);

            new_belief[new_serialized_state] = belief_state.second;
        }
        return new_belief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const
    {
        TBelief next_belief;
        number ag_id = belief.getCurrentAgentId();

        // Determine the next belief
        if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
        {
            // Call the function next State Serial Step which determine the next belief when the it's not the last agent
            next_belief = this->nextStateSerialStep(belief, action);
        }
        else
        {
            // Call the function next State Serial Last Agent which determine the next belief when the it's the last agent
            next_belief = this->nextStateSerialLastAgent(belief, action, obs);
        }
        return next_belief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, number t, std::shared_ptr<HSVI<TBelief, TAction>> hsvi) const
    {
        TBelief new_belief;

        // Select o* as in the paper
        TObservation selected_o;
        double max_o = -std::numeric_limits<double>::max(), tmp;

        // Go over all Serial Observation
        for (const auto &o : this->serialized_mpomdp_->getObsSpace(t)->getAll())
        {
            tmp = this->getObservationProbability(action, o, belief);
            // Call the function next State Serial Last Agent which determine the next belief when the it's the last agent
            auto tau = this->nextState(belief, action, o);

            tmp *= hsvi->do_excess(tau, 0, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                selected_o = o;
                new_belief = tau;
            }
        }
        return new_belief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteSpace<TAction>> SerializedBeliefMDP<TBelief, TAction, TObservation>::getActionSpaceAt(const TBelief &belief)
    {
        // Get the action Space of the current agent
        return this->serialized_mpomdp_->getActionSpace(belief.getCurrentAgentId());
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getReward(const TBelief &belief, const TAction &action) const
    {
        double r = 0;

        // Go over all element of the belief state
        for (const auto &belief_state : belief)
        {
            // \sum_{s} b(s)*r(s,a)
            r += belief_state.second * this->serialized_mpomdp_->getReward(belief_state.first, action);
        }
        return r;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getExpectedNextValue(std::shared_ptr<ValueFunction<TBelief, TAction>> value_function, const TBelief &belief, const TAction &action, number t) const
    {
        double exp_next_v = 0;

        // Go over all observation
        for (const auto &obs : this->serialized_mpomdp_->getObsSpace(t)->getAll())
        {
            TBelief next_belief = this->nextState(belief, action, obs);
            exp_next_v += this->getObservationProbability(action, obs, belief) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const
    {
        // Compute : sum_{s} b(s)* \sum_{s'} T(s,a,o,s')

        double proba = 0, tmp;

        //Go over all state in the belief
        for (const auto &belief_state : belief)
        {

            tmp = 0;
            auto serialized_state = belief_state.first;

            // Go over all the next Serial State possible
            for (auto next_serialized_state : this->serialized_mpomdp_->getReachableSerialStates(serialized_state, action))
            {
                tmp += this->serialized_mpomdp_->getDynamics(serialized_state, action, obs, next_serialized_state);
            }
            proba += tmp * belief_state.second;
        }
        return proba;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<SerializedMMDP> SerializedBeliefMDP<TBelief, TAction, TObservation>::toMDP()
    {
        return this->serialized_mpomdp_->toMDP();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> SerializedBeliefMDP<TBelief, TAction, TObservation>::toBeliefMDP()
    {
        return this->getptr();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> SerializedBeliefMDP<TBelief, TAction, TObservation>::getptr()
    {
        return SerializedBeliefMDP<TBelief, TAction, TObservation>::shared_from_this();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getDiscount(number horizon)
    {
        return this->serialized_mpomdp_->getDiscount(horizon);
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getWeightedDiscount(number horizon)
    {
        return std::pow(this->getDiscount(), horizon / this->serialized_mpomdp_->getNumAgents());
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::do_excess(double, double lb, double ub, double, double error, number horizon)
    {
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TAction SerializedBeliefMDP<TBelief, TAction, TObservation>::selectNextAction(const std::shared_ptr<ValueFunction<TBelief, TAction>> &, const std::shared_ptr<ValueFunction<TBelief, TAction>> &ub, const TBelief &s, number h)
    {
        return ub->getBestAction(s, h);
    }
} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::SerializedBeliefState>
    {
        typedef sdm::SerializedBeliefState argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::BaseBeliefState<sdm::SerializedState>>()(in);
        }
    };
}