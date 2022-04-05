#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/utils/linear_algebra/hyperplane/hyperplane.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/prunable_structure.hpp>
#include <sdm/utils/value_function/update_rule/vupdate_rule.hpp>

namespace sdm
{

    /**
     * @brief The interface for piece-wise linear convex value function.
     *
     * This kind of representation is usually used in HSVI to represent
     * the lower bound.
     *
     */
    class PWLCValueFunctionInterface : virtual public ValueFunctionInterface, public PrunableStructure
    {
    public:
        PWLCValueFunctionInterface(const std::shared_ptr<SolvableByDP> &world,
                                   const std::shared_ptr<Initializer> &initializer = nullptr,
                                   const std::shared_ptr<ActionSelectionInterface> &action = nullptr,
                                   int freq_pruning = -1);

        /**
         * @brief Add a hyperplane in the hyperplan set.
         *
         * This fonction will addd the hyperplan called new_hyperplan in the
         * set of plans at a specific time step.
         *
         * @param state the state
         * @param new_hyperplane the new hyperplane
         * @param t the timestep
         */
        virtual void addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &new_hyperplan, number t) = 0;

        virtual std::shared_ptr<Hyperplane> getHyperplaneAt(std::shared_ptr<State> state, number t) = 0;

        virtual std::vector<std::shared_ptr<Hyperplane>> getHyperplanesAt(std::shared_ptr<State> state, number t) = 0;

        virtual double getBeta(const std::shared_ptr<Hyperplane> &alpha, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t) = 0;

        virtual double getDefaultValue(number t) = 0;
    };
}