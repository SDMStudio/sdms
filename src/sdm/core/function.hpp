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

        TOutput operator()(const TInput &) const;
    };
} // namespace sdm
