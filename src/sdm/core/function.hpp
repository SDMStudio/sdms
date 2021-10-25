#pragma once
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/state/state.hpp>


/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <class TInput, class TOutput>
    class Function
    {
    public:
        using input_type = TInput;
        using output_type = TOutput;

        virtual ~Function() {}
        
        virtual output_type operator()(const input_type &) = 0;
    };

    template <class TInput1, class TInput2, class TOutput>
    class BinaryFunction : public Function<std::pair<TInput1, TInput2>, TOutput>
    {
    public:
        using input_type = std::pair<TInput1, TInput2>;
        using output_type = TOutput;

        virtual ~BinaryFunction() {}
        
        virtual output_type operator()(const TInput1 &, const TInput2 &) = 0;

        output_type operator()(const input_type &p_input)
        {
            return this->operator()(p_input.first, p_input.second);
        }
    };

    class BaseRelaxedValueFunction : public BinaryFunction<std::shared_ptr<State>, number, double>
    {
    public :
        virtual ~BaseRelaxedValueFunction() {}

        virtual double operator()(const std::shared_ptr<State> &ostate, number t) = 0;

        virtual bool isPomdpAvailable() = 0;
        virtual bool isMdpAvailable() = 0;

    };

    class ValueFunction;

    class RelaxedValueFunction : public BaseRelaxedValueFunction
    {
    public :

        virtual ~RelaxedValueFunction() {}

        virtual double operator()(const std::shared_ptr<State> &ostate, number t) = 0;

        virtual double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &ostate, number t) = 0;

        virtual bool isPomdpAvailable() = 0;
        virtual bool isMdpAvailable() = 0;

        virtual std::shared_ptr<ValueFunction> getRelaxation() = 0;

    };
} // namespace sdm
