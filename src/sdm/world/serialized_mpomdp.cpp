#include <sdm/world/serialized_mpomdp.hpp>

namespace sdm
{   
    SerializedMPOMDP::SerializedMPOMDP(std::shared_ptr<POMDPInterface> mpomdp)
    {}

    /**
     * @brief Get the reachable next states
     * 
     * @param state the state
     * @param action the action
     * @return the set of reachable states
     */
    std::set<std::shared_ptr<Observation>> SerializedMPOMDP::getAllObservations(number t) const
    {}

    /**
     * @brief Get the Reachablel Observations object
     * 
     * @param state 
     * @param action 
     * @param t 
     * @return std::set<std::shared_ptr<Observation>> 
     */
    std::set<std::shared_ptr<Observation>> SerializedMPOMDP::getReachablelObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {}

    /**
     * @brief Get the Obs Probability object
     * 
     * @param action 
     * @param next_state 
     * @param observation 
     * @param t 
     * @return double 
     */
    double SerializedMPOMDP::getObsProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {}

    /**
     * @brief Get the Dynamics object
     * 
     * @param state 
     * @param action 
     * @param next_state 
     * @param observation 
     * @param t 
     * @return double 
     */
    double SerializedMPOMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {}
}