// #pragma once

// #include <sdm/types.hpp>
// #include <sdm/core/state/interface/belief_interface.hpp>
// #include <sdm/core/state/base_state.hpp>
// #include <sdm/core/space/space.hpp>


// namespace sdm
// {
//     class ObservationStateBeliefInterface : virtual public BeliefInterface
//     {
//     public:
//         virtual double getProbability(const std::shared_ptr<State> &observation) const = 0;
//         virtual double getProbability(const std::shared_ptr<Observation> &observation) const = 0;
//         virtual double getProbability(const std::shared_ptr<Observation> &observation, const std::shared_ptr<State> &state) const = 0;

//         virtual void setProbability(const std::shared_ptr<State> &obsevation_belief, double proba) = 0;
//         virtual void setProbability(const std::shared_ptr<Observation> &observation, const std::shared_ptr<BeliefInterface> &belief, double proba) = 0;

//         virtual void addProbability(const std::shared_ptr<State> &obsevation_belief, double proba) = 0;
//         virtual void addProbability(const std::shared_ptr<Observation> &observation, const std::shared_ptr<BeliefInterface> &belief, double proba) = 0;

//         virtual double operator-(const std::shared_ptr<BeliefInterface> &other) const = 0;

//         virtual double minus(const std::shared_ptr<BeliefInterface> &other) const = 0;

//         virtual Pair<std::shared_ptr<Observation>, std::shared_ptr<BeliefInterface>> sampleObservationBelief() = 0;

//         /**
//          * @brief Get the set of joint histories that are in the support of the occupancy state.
//          * @return the possible joint hitories
//          */
//         virtual const std::set<std::shared_ptr<Observation>> &getObservations() const = 0;

//         /**
//          * @brief Get the set of beliefs at a given joint history
//          * 
//          * @param jhistory 
//          * @return const std::set<std::shared_ptr<BeliefInterface>>& 
//          */
//         virtual std::shared_ptr<BeliefInterface> getBeliefAt(const std::shared_ptr<Observation> &observation) const = 0;

//         virtual void finalize() = 0;
//     };
// }