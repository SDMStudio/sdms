#pragma once

#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/utils/linear_algebra/hyperplane.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>
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
    class ParametricQValueFunction : public QValueFunction, public PWLCValueFunctionInterface
    {
    public:
        using Container = std::shared_ptr<Hyperplane>;

        /**
         * @brief Construct a piece-wise linear convex q-value function
         *
         * @param world the world
         * @param initializer the initializer
         * @param action the action selection
         * @param update_operator the update operator
         */
        ParametricQValueFunction(const std::shared_ptr<SolvableByDP> &world,
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

        /**
         * @brief Get the q-value.
         *
         * @param x the state
         * @param o the history
         * @param u the action
         * @param t the time step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, number t);

        void setQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, double value, number t);

        void addHyperplaneAt(const std::shared_ptr<State> &, const std::shared_ptr<State> &new_hyperplane, number t);

        std::shared_ptr<State> getHyperplaneAt(const std::shared_ptr<State> &, number t);
        
        std::vector<std::shared_ptr<State>> getHyperplanesAt(const std::shared_ptr<State> &, number t);

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

        friend std::ostream &operator<<(std::ostream &os, ParametricQValueFunction &vf)
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
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;

        std::shared_ptr<OccupancyMDP> oMDP;
    };

} // namespace sdm