#pragma once
#include <sdm/utils/struct/pair.hpp>


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

    template <typename TOccupancyState>
    class BaseRelaxedValueFunction : public BinaryFunction<TOccupancyState, number, double>
    {
    public :
        virtual ~BaseRelaxedValueFunction() {}

        virtual double operator()(const TOccupancyState &ostate, const number &tau) = 0;

        virtual bool isPomdpAvailable() = 0;
        virtual bool isMdpAvailable() = 0;

    };

    template <typename TState, typename TOccupancyState>
    class RelaxedValueFunction : public BaseRelaxedValueFunction<TOccupancyState>, public BaseRelaxedValueFunction<Pair<TState, number>>
    {
    public :
        virtual ~RelaxedValueFunction() {}

        virtual double operator()(const TOccupancyState &ostate, const number &tau) = 0;

        virtual double operator()(const Pair<TState, number> &ostate, const number &tau) = 0;

        virtual bool isPomdpAvailable() = 0;
        virtual bool isMdpAvailable() = 0;

    };
} // namespace sdm
