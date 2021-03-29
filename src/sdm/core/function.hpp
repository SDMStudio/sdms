#pragma once

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

        virtual output_type operator()(const input_type &) = 0;
    };

    template <class TInput1, class TInput2, class TOutput>
    class BinaryFunction : public Function<std::pair<TInput1, TInput2>, TOutput>
    {
    public:
        using input_type = std::pair<TInput1, TInput2>;
        using output_type = TOutput;

        virtual output_type operator()(const TInput1 &, const TInput2 &) = 0;

        output_type operator()(const input_type &p_input)
        {
            return this->operator()(p_input.first, p_input.second);
        }
    };
} // namespace sdm
