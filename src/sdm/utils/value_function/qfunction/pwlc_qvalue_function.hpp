#pragma once

#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/utils/linear_algebra/hyperplane.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief Q-value function instance represented by a piecewise linear and convex function.
     *
     * This representation is specific to the resolution of decentralized POMDP. A linear function
     * is assigned to each cluster of occupancy states (close to a granularity coefficient).
     *
     */
    class PWLCQValueFunction : public QValueFunction, public PWLCValueFunctionInterface
    {
    public:
        /**
         * @brief The precision used to assign a representant to occupancy states.
         */
        static double GRANULARITY;
        static double GRANULARITY_START;
        static double GRANULARITY_END;

        struct Equal
        {
            virtual bool operator()(const std::shared_ptr<State> &left, const std::shared_ptr<State> &right) const
            {
                // if left or right is a nullptr, then return false
                // if ((left == nullptr) ^ (right == nullptr))
                //     return false;
                // return ((left == right) || left->isEqual(right, PWLCQValueFunction::GRANULARITY));

                double norm_1 = 0., additional = 1., proba_right;
                auto ostate_left = left->toOccupancyState(), ostate_right = right->toOccupancyState();
                // For all points in the support
                for (const auto &jhistory : ostate_left->getJointHistories())
                {
                    // For all states in the corresponding belief
                    for (const auto &state : ostate_left->getBeliefAt(jhistory)->getStates())
                    {
                        proba_right = ostate_right->getProbability(jhistory, state);
                        additional -= proba_right;
                        norm_1 += std::abs(ostate_left->getProbability(jhistory, state) - proba_right);
                    }
                }

                return (((norm_1 + additional) / 2) - 1e-5 <= PWLCQValueFunction::GRANULARITY);
            }
        };

        struct Hash
        {
            virtual size_t operator()(const std::shared_ptr<State> &state) const
            {
                // return (state) ? 0 : state->hash(PWLCQValueFunction::GRANULARITY);
                size_t seed = 0;
                // double inverse_of_precision = 1. / PWLCQValueFunction::GRANULARITY;
                // std::map<std::shared_ptr<sdm::State>, double> ordered(in.begin(), in.end());
                // std::vector<int> rounded;
                // for (const auto &pair_jhist_proba : ordered)
                // {
                //     double round = lround(inverse_of_precision * pair_jhist_proba.second);
                //     if (round > 0)
                //     {
                //         sdm::hash_combine(seed, pair_jhist_proba.first);
                //         // sdm::hash_combine(seed, in.getBeliefAt(pair_jhist_proba.first->toHistory()->toJointHistory()));
                //         rounded.push_back(lround(inverse_of_precision * pair_jhist_proba.second));
                //     }
                // }
                // for (const auto &v : rounded)
                // {
                //     // Combine the hash of the current vector with the hashes of the previous ones
                //     sdm::hash_combine(seed, v);
                // }
                return seed;
            }
        };

        // using Hyperplane = BaseState<MappedVector<std::tuple<std::shared_ptr<State>, std::shared_ptr<HistoryInterface>, std::shared_ptr<Action>>>>;
        using Container = std::unordered_map<std::shared_ptr<State>, std::shared_ptr<Hyperplane>, Hash, Equal>;

        /**
         * @brief Construct a piece-wise linear convex q-value function
         *
         * @param world the world
         * @param initializer the initializer
         * @param action the action selection
         * @param update_operator the update operator
         */
        PWLCQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                           const std::shared_ptr<Initializer> &initializer = nullptr,
                           const std::shared_ptr<ActionSelectionInterface> &action = nullptr,
                           const std::shared_ptr<PWLCQUpdateOperator> &update_operator = nullptr);
        /**
         * @brief Initialize the value function
         */
        void initialize();

        /**
         * @brief Initialize the value function with a default value
         */
        void initialize(double v, number t = 0);

        /**
         * @brief Get the q-value at a specific state and time step.
         *
         * @param state the state
         * @param t the time step
         * @return the action value vector
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Get the q-value at a specific state, action and time step.
         *
         * @param state the state
         * @param action the action
         * @param t the time step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
        double getQValueAt(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, number t);

        /**
         * @brief Get the q-value.
         *
         * @param x the state
         * @param o the history
         * @param u the action
         * @param t the time step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t);

        void addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<State> &new_hyperplane, number t);

        std::shared_ptr<State> getHyperplaneAt(std::shared_ptr<State> state, number t);

        std::vector<std::shared_ptr<State>> getHyperplanesAt(std::shared_ptr<State> state, number t);

        double getBeta(const std::shared_ptr<State> &belief_state, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Prune unecessary components of the value function.
         *
         * This function will prune the dominated hyperplanes.
         *
         * @param t the time step
         */
        void prune(number t);

        double getDefaultValue(number t);

        Container getRepresentation(number t);

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, PWLCQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

        /**
         * @brief The data structure storing the value function representation.
         */
        std::vector<Container> representation;

    protected:
        /**
         * @brief The value by default.
         */
        std::vector<std::shared_ptr<Hyperplane>> default_hyperplane;

        std::vector<double> granularity_per_horizon;

        /**
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;

        std::shared_ptr<OccupancyMDP> oMDP;
    };

} // namespace sdm