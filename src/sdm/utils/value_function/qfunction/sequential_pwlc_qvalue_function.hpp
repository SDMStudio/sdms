#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/bbeta.hpp>
#include <sdm/utils/linear_algebra/hyperplane/obeta.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
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
    template <typename TBetaVector>
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
                return left->isEqual(right, PWLCQValueFunction::GRANULARITY);
            }
        };

        struct Hash
        {
            virtual bool operator()(const std::shared_ptr<State> &state) const
            {
                return state->hash(PWLCQValueFunction::GRANULARITY);
            }
        };

        using SequentialContainer = std::unordered_map<std::shared_ptr<State>, std::shared_ptr<BetaVector>>;
        using SimultaneousContainer = std::unordered_map<std::shared_ptr<State>, std::shared_ptr<BetaVector>, Hash, Equal>;

        /**
         * @brief Construct a piece-wise linear convex q-value function
         *
         * @param world the world
         * @param initializer the initializer
         * @param action the action selection
         * @param update_rule the update operator
         */
        PWLCQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                           const std::shared_ptr<Initializer> &initializer = nullptr,
                           const std::shared_ptr<ActionSelectionInterface> &action = nullptr,
                           const std::shared_ptr<PWLCQUpdateRule> &update_rule = nullptr);
        /**
         * @brief Initialize the value function
         */
        void initialize();

        /**
         * @brief Initialize the value function with a default value
         */
        void initialize(double v, number t = 0);

        /**
         * @brief Get the q-value at a specific state, action and time step.
         *
         * @param state the state
         * @param action the action
         * @param t the time step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

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

        void addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &new_hyperplane, number t);

        std::shared_ptr<Hyperplane> getHyperplaneAt(std::shared_ptr<State> state, number t);

        std::vector<std::shared_ptr<Hyperplane>> getHyperplanesAt(std::shared_ptr<State> state, number t);

        double getBeta(const std::shared_ptr<Hyperplane> &belief_state, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Get the q-value at a specific state and time step.
         *
         * @param state the state
         * @param t the time step
         * @return the action value vector
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const std::shared_ptr<State> &state, number t);

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

        bool isLastAgent(number t) const;
        number getSimIndex(number t) const;
        number getSerialIndex(number t) const;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, PWLCQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief The data structure storing the value function representation.
         */
        std::vector<SequentialContainer> sequential_representation;
        std::vector<SimultaneousContainer> simultaneous_representation;

        /**
         * @brief The value by default.
         */
        std::vector<std::shared_ptr<BetaVector>> default_hyperplane;

        /**
         * @brief Keep the granularity to apply to each time step.
         *
         */
        std::vector<double> granularity_per_horizon;

        /**
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;
    };

} // namespace sdm

#include <sdm/utils/value_function/qfunction/sequential_pwlc_qvalue_function.tpp>