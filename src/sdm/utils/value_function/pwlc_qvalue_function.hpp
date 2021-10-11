#pragma once

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/utils/value_function/initializer/initializers.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
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
    class PieceWiseLinearConvexQValueFunction : public QValueFunction
    {
    protected:
        /**
         * @brief The precision used to assign a representant to occupancy states.
         */
        static double GRANULARITY;

        struct Equal
        {
            virtual bool operator()(const OccupancyState &left, const OccupancyState &right) const
            {
                return (left.isEqual(right, PieceWiseLinearConvexQValueFunction::GRANULARITY) || right.isEqual(left, PieceWiseLinearConvexQValueFunction::GRANULARITY));
            }
        };

        struct Hash
        {
            virtual size_t operator()(const OccupancyState &item) const
            {
                return std::hash<sdm::OccupancyState>()(item, PieceWiseLinearConvexQValueFunction::GRANULARITY);
            }
        };

        using PSI = std::unordered_map<OccupancyState, TabularQValueFunction, Hash, Equal>;

        /**
         * @brief The problem to be solved.
         * 
         */
        std::shared_ptr<OccupancyMDP> omdp;

        /**
         * @brief The data structure storing the value function representation. 
         */
        std::vector<PSI> representation;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer> initializer_;

        /**
         * @brief The value by default.
         */
        double default_value_, learning_rate_;

        /**
         * @brief Get a pointer on a linear value function at an occupancy state
         * 
         * @param state the state
         * @param t the time step
         * @return the address of the q-value function corresponding to an occupancy state 
         */
        TabularQValueFunction *getQ(const std::shared_ptr<State> &state, number t);

    public:
        PieceWiseLinearConvexQValueFunction(const std::shared_ptr<OccupancyMDP> &omdp, number horizon = 0, double learning_rate = 0.1, double default_value = 0.);

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
         * @brief Update the value at a given state
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double delta, number t = 0);

        int getNumStates() const;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, PieceWiseLinearConvexQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }
    };

} // namespace sdm