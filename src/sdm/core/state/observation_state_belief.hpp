// #pragma once
// #include <sdm/types.hpp>
// #include <sdm/core/joint.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/belief_state.hpp>
// #include <sdm/core/state/interface/observation_state_belief_interface.hpp>
// #include <sdm/core/state/interface/belief_interface.hpp>
// #include <sdm/utils/struct/recursive_map.hpp>
// #include <sdm/utils/linear_algebra/mapped_vector.hpp>

// namespace sdm
// {

//     /**
//      * @brief An occupancy state refers to the whole knowledge that a central planner can have access to take decisions.
//      * 
//      * @tparam TState refers to a number
//      * @tparam TJointHistory_p refers to a joint histories
//      */
//     class ObservationStateBelief : virtual public ObservationStateBeliefInterface,
//                            virtual public Belief
//     {
//     public:
//         static double PRECISION;

//         ObservationStateBelief();
//         ObservationStateBelief(number num_agents);
//         ObservationStateBelief(const ObservationStateBelief &copy);

//         size_t hash() const;

//         bool operator==(const std::shared_ptr<State> &other) const;
//         bool operator==(const ObservationStateBelief &other) const;

//         double getProbability(const std::shared_ptr<State> &joint_history) const;
//         double getProbability(const std::shared_ptr<Observation> &joint_history) const;
//         double getProbability(const std::shared_ptr<Observation> &joint_history, const std::shared_ptr<State> &state) const;

//         void setProbability(const std::shared_ptr<State> &joint_history, double proba);
//         void setProbability(const std::shared_ptr<Observation> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);

//         void addProbability(const std::shared_ptr<State> &joint_history, double proba);
//         void addProbability(const std::shared_ptr<Observation> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);

//         Pair<std::shared_ptr<Observation>, std::shared_ptr<BeliefInterface>> sampleObservationBelief();

//         /**
//          * @brief Get the set of joint histories that are in the support of the occupancy state.
//          * @return the possible joint hitories
//          */
//         const std::set<std::shared_ptr<Observation>> &getObservations() const;

//         /**
//          * @brief Get the belief corresponding to the given joint historiy.

//          * @return the belief
//          */
//         std::shared_ptr<BeliefInterface> getBeliefAt(const std::shared_ptr<Observation> &observation) const;

//         /**
//          * @brief Set the belief corresponding to the given joint historiy.
//          * 
//          * @param observation the joint history
//          * @param belief the corresponding belief 
//          */
//         void setBeliefAt(const std::shared_ptr<Observation> &observation, const std::shared_ptr<BeliefInterface> &belief);

//         void finalize();

//         TypeState getTypeState() const;

//         std::shared_ptr<ObservationStateBelief> getptr();

//         std::string str() const;

//         double operator^(const std::shared_ptr<BeliefInterface> &other) const;
//         bool operator==(const std::shared_ptr<BeliefInterface> &other) const;

//         double operator<(const ObservationStateBelief &other) const;
//         double operator<(const std::shared_ptr<BeliefInterface> &other) const;

//         double operator-(const std::shared_ptr<BeliefInterface> &other) const;

//         double minus(const std::shared_ptr<BeliefInterface> &other) const;

//         void normalize();

//     protected:
//         /**
//          * @brief the number of agents 
//          */
//         number num_agents_ = 1;

//         /**
//          * @brief space of all reachable states, those in the support of the occupancy state
//          * @comment: Should not be used since there are to much possible wrt each joint history, belief states whould have been a better choice.
//          */
//         std::set<std::shared_ptr<BeliefInterface>> list_beliefs_;

//         /**
//          * @brief space of joint histories
//          */
//         std::set<std::shared_ptr<Observation>> list_observation_;

//     };
// } // namespace sdm

// namespace std
// {

//     // template <>
//     // struct hash<sdm::ObservationStateBelief>
//     // {
//     //     typedef sdm::ObservationStateBelief argument_type;
//     //     typedef std::size_t result_type;
//     //     inline result_type operator()(const argument_type &in) const
//     //     {
//     //         clock_t t_begin = clock();

//     //         size_t seed = 0;
//     //         double inverse_of_precision = 1. / sdm::ObservationStateBelief::PRECISION;
//     //         std::map<std::shared_ptr<sdm::State>, double> ordered(in.begin(), in.end());
//     //         std::vector<int> rounded;
//     //         for (const auto &pair_jhist_proba : ordered)
//     //         {
//     //             sdm::hash_combine(seed, pair_jhist_proba.first);
//     //             // sdm::hash_combine(seed, in.getBeliefAt(pair_jhist_proba.first->toHistory()->toJointHistory()));
//     //             rounded.push_back(lround(inverse_of_precision * pair_jhist_proba.second));
//     //         }
//     //         for (const auto &v : rounded)
//     //         {
//     //             //Combine the hash of the current vector with the hashes of the previous ones
//     //             sdm::hash_combine(seed, v);
//     //         }
//     //         sdm::ObservationStateBelief::TIME_IN_HASH += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//     //         return seed;
//     //         // return std::hash<sdm::MappedVector<std::shared_ptr<sdm::State>>>()(in, sdm::ObservationStateBelief::PRECISION);
//     //     }
//     // };
// }