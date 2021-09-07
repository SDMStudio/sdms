// /**
//  * @file belief_state.cpp
//  * @author David Albert (david.albert@insa-lyon.fr)
//  * @brief 
//  * @version 1.0
//  * @date 29/03/2021
//  * 
//  * @copyright Copyright (c) 2021
//  * 
//  */
// #pragma once

// #include <algorithm>

// #include <sdm/types.hpp>
// #include <sdm/utils/struct/pair.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/interface/belief_interface.hpp>
// #include <sdm/utils/linear_algebra/vector.hpp>
// #include <sdm/utils/linear_algebra/mapped_vector.hpp>
// #include <sdm/utils/linear_algebra/sdms_vector.hpp>
// #include <sdm/core/distribution.hpp>

// namespace sdm
// {
//   class JointBelief : virtual public Joint<std::shared_ptr<BeliefInterface>>,
//                       public State
//   {
//   public:
//     static double PRECISION;

//     JointBelief();
//     virtual ~JointBelief();

//     double getProbability(number agent_id, const std::shared_ptr<State> &state) const;
//     void setProbability(number agent_id, const std::shared_ptr<State> &state, double proba);
//     void addProbability(number agent_id, const std::shared_ptr<State> &state, double proba);

//     void normalizeBelief(double norm_1);

//     std::string str() const;

//     void finalize();

//     friend std::ostream &operator<<(std::ostream &os, const Belief &belief)
//     {
//       os << belief.str();
//       return os;
//     }

//     template <class Archive>
//     void serialize(Archive &archive, const unsigned int)
//     {
//       archive &boost::serialization::base_object<MappedVector<std::shared_ptr<State>, double>>(*this);
//     }
//   };
// } // namespace sdm

// namespace std
// {
//   template <>
//   struct hash<sdm::JointBelief>
//   {
//     typedef sdm::JointBelief argument_type;
//     typedef std::size_t result_type;
//     inline result_type operator()(const argument_type &in) const
//     {
//       return std::hash<sdm::MappedVector<std::shared_ptr<sdm::State>>>()(in, sdm::JointBelief::PRECISION);
//     }
//   };
// }