/**
 * @file action.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for interface action class
 * @version 0.1
 * @date 11/12/2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/item.hpp>

namespace sdm
{
    class DecisionRule;
    class JointDeterministicDecisionRule;
    template <typename T>
    class Joint;
    using JointAction = Joint<std::shared_ptr<Action>>;

    /**
     * @brief A public interface for actions.
     *
     * Any class inheriting from this interface will be considered as generic action for algorithms.
     * Consider sections [Theoritical Background](https://aldavid.gitlabpages.inria.fr/sdms/tutorials/theory.html) and [Algorithms](https://aldavid.gitlabpages.inria.fr/sdms/tutorials/algorithms/) for more information.
     *
     */
    class Action : public Item
    {
    public:
        using base = Action;

        virtual ~Action() {}
        /**
         * @brief Get the hash of the state.
         *
         * The hash is used in tabular value functions in order to compare efficiently two states.
         * This function must be reimplemented in inherited classes.
         *
         * @return size_t the hash code
         */
        virtual size_t hash(double precision = -1) const;

        /**
         * @brief Check equality between two states.
         *
         * This function must be implemented in inherited classes.
         *
         * @param other the state to be compared to current state
         * @return true if states are equal
         * @return false if they are different
         */
        virtual bool isEqual(const std::shared_ptr<Action> &other, double precision = -1) const;

        /**
         * @brief Get string representation of the action.
         */
        virtual std::string str() const = 0;

        /** @brief Cast the action into a decision rule. */
        virtual std::shared_ptr<DecisionRule> toDecisionRule();

        /** @brief Cast the action into a joint decision rule. */
        virtual std::shared_ptr<JointDeterministicDecisionRule> toJointDeterministicDecisionRule();

        /** @brief Cast the action into a joint decision rule. */
        std::shared_ptr<JointAction> toJointAction();
    };
} // namespace sdm