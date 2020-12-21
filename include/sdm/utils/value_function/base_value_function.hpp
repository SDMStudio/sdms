/**
 * @file base_value_function.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Defines the value function base.
 * @version 0.1
 * @date 16/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <map>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
 * @class BaseValueFunction 
 *
 * @brief 
 * 
 */
    template <typename TState, typename vType, template <typename> typename TStruct>
    class BaseValueFunction
    {
    protected:
        TStruct<TState, vType> value;

    public:
        BaseValueFunction(TStruct init_values);
        BaseValueFunction(TStruct init_values);

        vType getValueAt(TState state) const;

        void updateValueAt(TState state, vType value);
    };


    template <typename TState, template <typename> typename TStruct = std::map>
    using ValueFunction = BaseValueFunction<TState, double, TStruct>;

    template <typename TState, typename TAction, template <typename> typename TStruct = std::map>
    using QValueFunction = ValueFunction<std::pair<TState, TAction>, TStruct>;

    template <typename TState, typename TAction, template <typename> typename TStruct = std::map>
    using QValueFunction2 = BaseValueFunction<TState, ValueFunction<TAction, TStruct>, TStruct>;

    template <typename TState, template <typename> typename TStruct = std::map>
    using QValueFunction3 = BaseValueFunction<TState, Vector, TStruct>;

    template <class TModel>
    using DeepQValueFunction = BaseValueFunction<Tensor, Tensor, TModel>;

    template <class TModel>
    using DeepValueFunction = ValueFunction<Tensor, TModel>;

} // namespace sdm