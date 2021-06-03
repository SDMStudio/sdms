#pragma once

#include <../toulbar2/src/toulbar2lib.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>
#include <sdm/utils/value_function/variable_naming.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <typename TVector, typename TAction, typename TValue = double>
    class MaxPlanValueFunctionWCSP :  public MaxPlanValueFunction<TVector, TAction, TValue>, Var
    {
    public:

        MaxPlanValueFunctionWCSP();

        /**
         * @brief Construct a new MaxPlane Value Function object
         * 
         * @param std::shared_ptr<SolvableByHSVI<TState, TAction>> : problem 
         * @param number : horizon 
         * @param std::shared_ptr<Initializer<TState, TAction>> : initializer 
         * @param int frequency of the pruning 
         */
        MaxPlanValueFunctionWCSP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> , int , std::shared_ptr<Initializer<TVector, TAction>>, int = 10 );

        /**
         * @brief Construct a new MaxPlane Value Function object
         * 
         * @param std::shared_ptr<SolvableByHSVI<TState, TAction>> : problem 
         * @param number : horizon 
         * @param TValue : initializer 
         * @param int frequency of the pruning 
         */
        MaxPlanValueFunctionWCSP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> , int = 0, TValue = 0., int = 10);
        
        // /**
        //  * @brief Sets the joint and individual types
        //  * 
        //  * @param const std::shared_ptr<State>&
        //  */
        // void setJointTypes(const std::shared_ptr<State> &);

        // //! \fn       vvoid setOffSet(number);
        // //! \param    number the offset integer value.
        // //! \param    Sets the offset value.
        // void setOffSet(number);

        // //! \fn         std::shared_ptr<Action> getGreedyActionAt(const std::shared_ptr<State>&, double&, const std::shared_ptr<value_function> &);
        // //! \param[in]  const std::shared_ptr<State>&,
        // //! \param[out] double& defines the overall value to be stored, i.e., performance of the solution.
        // //! \param[in]  const std::shared_ptr<value_function> & defines  the lower or upper default bound
        // //! \return     Returns the joint decision rule, i.e., mapping individual types to individual actions.
        // //! \brief      This method implements the greedy backup operator using exhaustive enumeration of all decision rules.
        // //!             We shall only consider the simultaneous approach here, since the sequential variant would be unnecessary complex:
        // //!             @f{align*}
        // //!                 a^*_\tau  &\in \argmax_{a_\tau\in \mathcal{A}_\tau} [R(s_\tau, a_\tau) + \gamma V_{\tau+1}(T(s_\tau, a_\tau)).
        // //!             @f}
        // //!             To do so, we make use of toulbar2 library instance and solve it using @WeightedCSPSolver, thereby circumventing the exhaustive enumeration.
        // //!
        TAction getGreedyActionAt(const TVector &, double &);

    protected :

        /**
         * @brief defines the maximum value in the domain of the payoff function
         * 
         */
        double max;

        /**
         * @brief factor used to convert real values into integer costs
         * 
         */
        long offset = 1000000000000;

        /**
         * @brief Returns a cost value
         * 
         * @param double defines a real value
         */
        int getCost(double);

    };
}