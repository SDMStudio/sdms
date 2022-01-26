#include <sdm/world/transformed_mpomdp.hpp>

namespace sdm
{
    TransformedMPOMDP::TransformedMPOMDP(const std::shared_ptr<MPOMDPInterface> &mpomdp) : mpomdp_(mpomdp) {}

    number TransformedMPOMDP::getNumAgents() const
    {
        return this->mpomdp_->getNumAgents();
    }

    number TransformedMPOMDP::getHorizon() const
    {
        return this->mpomdp_->getHorizon();
    }

    double TransformedMPOMDP::getDiscount(number t) const
    {
        return this->mpomdp_->getDiscount(t);
    }

    double TransformedMPOMDP::getWeightedDiscount(number t) const
    {
        return this->mpomdp_->getWeightedDiscount(t);
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> TransformedMPOMDP::getStartDistribution() const
    {
        return this->mpomdp_->getStartDistribution();
    }

    std::shared_ptr<Space> TransformedMPOMDP::getStateSpace(number t) const
    {
        return this->mpomdp_->getStateSpace(t);
    }

    double TransformedMPOMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->mpomdp_->getReward(state, action, t);
    }

    double TransformedMPOMDP::getMinReward(number t) const
    {
        return this->mpomdp_->getMinReward(t);
    }
    double TransformedMPOMDP::getMaxReward(number t) const
    {
        return this->mpomdp_->getMaxReward(t);
    }

    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> TransformedMPOMDP::step(std::shared_ptr<Action> action)
    {
        return this->mpomdp_->step(action);
    }
    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> TransformedMPOMDP::step(std::shared_ptr<Action> action, bool increment_timestep)
    {
        return this->mpomdp_->step(action, increment_timestep);
    }

    double TransformedMPOMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->mpomdp_->getTransitionProbability(state, action, next_state, t);
    }

    std::set<std::shared_ptr<State>> TransformedMPOMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->mpomdp_->getReachableStates(state, action, t);
    }

    void TransformedMPOMDP::setInternalState(std::shared_ptr<State> state)
    {
        return this->mpomdp_->setInternalState(state);
    }

    std::shared_ptr<State> TransformedMPOMDP::getInternalState() const
    {
        return this->mpomdp_->getInternalState();
    }

    std::set<std::shared_ptr<Observation>> TransformedMPOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->mpomdp_->getReachableObservations(state, action, next_state, t);
    }

    double TransformedMPOMDP::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        return this->mpomdp_->getObservationProbability(state, action, next_state, observation, t);
    }

    double TransformedMPOMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        return this->getTransitionProbability(state, action, next_state, t) * this->getObservationProbability(state, action, next_state, observation, t);
    }

    std::shared_ptr<Space> TransformedMPOMDP::getActionSpace(number t) const
    {
        return this->mpomdp_->getActionSpace(t);
    }

    std::shared_ptr<Space> TransformedMPOMDP::getActionSpace(number agent_id, number t) const
    {
        return this->mpomdp_->getActionSpace(agent_id, t);
    }

    std::shared_ptr<Space> TransformedMPOMDP::getActionSpaceAt(const std::shared_ptr<State> &observation, number t)
    {
        return this->mpomdp_->getActionSpaceAt(observation, t);
    }

    std::shared_ptr<Space> TransformedMPOMDP::getObservationSpace(number agent_id, number t) const
    {
        return this->mpomdp_->getObservationSpace(agent_id, t);
    }

    std::shared_ptr<Space> TransformedMPOMDP::getObservationSpace(number t) const
    {
        return this->mpomdp_->getObservationSpace(t);
    }


    std::shared_ptr<Action> TransformedMPOMDP::getRandomAction(const std::shared_ptr<State> &observation, number t)
    {
        return this->mpomdp_->getRandomAction(observation, t);
    }

    std::shared_ptr<State> TransformedMPOMDP::reset()
    {
        return this->mpomdp_->reset();
    }

} // namespace sdm
