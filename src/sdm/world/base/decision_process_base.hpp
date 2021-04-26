/**
 * @file decision_process_base.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File of the decision process base class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/base/stochastic_process_base.hpp>

namespace sdm
{

    /**
     * @brief This class is the base class for decision processes. It contains base spaces required in all decision processes.
     * 
     * @tparam TStateSpace the state space
     * @tparam TActionSpace  the action space
     * @tparam TDistrib the initial distribution
     */
    template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    class DecisionProcessBase : public virtual StochasticProcessBase<TStateSpace, TDistrib>
    {
    public:
        using action_space_type = TActionSpace;
        using action_type = typename TActionSpace::value_type;

        DecisionProcessBase();
        DecisionProcessBase(std::shared_ptr<TStateSpace>, std::shared_ptr<TActionSpace>);
        DecisionProcessBase(std::shared_ptr<TStateSpace>, std::shared_ptr<TActionSpace>, TDistrib, number = 0, double = 0.9, Criterion = Criterion::REW_MAX);

        /**
         * @brief Get the filename
         * 
         * @return std::string 
         */
        std::string getFileName();

        /**
         * @brief Set the filename
         * 
         * @param filename the filename that permit to build this process
         */
        void setFileName(std::string);

        /**
         * @brief Get the bound
         * 
         * @return the bound
         */
        double getBound();

        /**
         * @brief Set the bound
         * 
         * @param bound the bound
         */
        void setBound(double);

        /**
         * @brief Get the criterion. Can be of two types : REW_MAX (reward maximisation) or COST_MIN (cost minimization).
         * 
         * @return Criterion the current criterion
         */
        Criterion getCriterion();

        /**
         * @brief Set the criterion
         * @param criterion the criterion to use. Can be of two types : (0) REW_MAX (reward maximisation) or (1) COST_MIN (cost minimization)
         * 
         */
        void setCriterion(Criterion);

        /**
         * @brief Get the discount factor
         * 
         * @return the discount factor used
         */
        double getDiscount();

        /**
         * @brief Set the discount factor
         * 
         * @param discount the discount factor to use.
         */
        void setDiscount(double);

        /**
         * @brief Get the planning horizon
         */
        number getPlanningHorizon();

        /**
         * @brief Set the planning horizon
         */
        void setPlanningHorizon(number);

        /**
         * \brief Getter for the action space
         */
        std::shared_ptr<TActionSpace> getActionSpace() const;

        /**
         * @brief Set the action space
         * 
         */
        void setActionSpace(std::shared_ptr<TActionSpace>);

    protected:
        /**
         * @brief Action space for each agent.
         */
        std::shared_ptr<TActionSpace> action_space_;

        /**
         * @brief planning horizon
         */
        number planning_horizon_ = 0;

        /**
         * @brief factor used to discount rewards (respectively costs) in the future.
         */
        double discount_ = 1.0, bound_;

        /**
         * @brief type of optimization problem, e.g., reward maximazation or cost minimization.
         */
        Criterion criterion_ = Criterion::REW_MAX;

        /**
         * @brief name of the file that generates the environment
         */
        std::string filename_;
    };

} // namespace sdm
#include <sdm/world/base/decision_process_base.tpp>
