#include <sdm/world/belief_mdp.hpp>

namespace sdm
{
    BeliefMDP::BeliefMDP(std::shared_ptr<DecPOMDP> underlying_pomdp) : hidden_mdp(underlying_pomdp)
    {
        current_belief = underlying_pomdp->getStartDistrib();
    }

    Vector BeliefMDP::getBelief() const
    {
        return this->current_belief;
    }

    Vector BeliefMDP::getNextBelief(Vector belief, number action, number obs) const
    {
        Vector next_belief(this->hidden_mdp->getNumStates());

        for (number nstate = 0; nstate < this->hidden_mdp->getNumStates(); nstate++)
        {
            double belief_transition = 0.;
            for (number cstate = 0; cstate < this->hidden_mdp->getNumStates(); cstate++)
            {
                belief_transition += this->hidden_mdp->getTransitionProba(cstate, action, nstate) * belief[cstate];
            }
            // TODO : nu normalized parameter
            next_belief[nstate] = nu * this->hidden_mdp->getObservationProbability(action, obs, nstate) * belief_transition;
        }

        return next_belief;
    }

    double BeliefMDP::getReward(Vector belief, number action) const
    {
        double belief_based_rew = 0.;

        for (number i = 0; i < this->hidden_mdp->getNumStates(); i++)
        {
            // TODO : create POMDP class
            belief_based_rew += this->hidden_mdp->getReward(i, action) * belief[i];
        }
        return belief_based_rew;
    }

    double BeliefMDP::getObservationProbability(number action, number obs, Vector belief) const
    {
        double obs_proba = 0.;
        for (number i = 0; i < this->hidden_mdp->getNumStates(); i++)
        {
            obs_proba += this->hidden_mdp->getObservationProbability(action, obs, i) * belief[i];
        }
        return obs_proba;
    }
} // namespace sdm