/**
 * @file initializers.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This file contains the initializer factory class.
 * @version 1.0
 * @date 24/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <map>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, template <typename TS, typename TA> class TValueFunction>
    std::shared_ptr<ValueFunction<TState, TAction>> createInstance() { return std::shared_ptr<TValueFunction<TState, TAction>>(new TValueFunction<TState, TAction>); }

    /**
     * @brief The ValueFunctionFactory class facilitates users to interact and instanciate value functions.   
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class ValueFunctionFactory
    {
    protected:
        typedef std::map<std::string, std::shared_ptr<Initializer<TState, TAction>> (*)()> map_type;
        static inline map_type registry_ = {
            {"TabularValueFunction", &createInstance<TState, TAction, TabularValueFunction>},
            {"MappedValueFunction", &createInstance<TState, TAction, MappedValueFunction>},
        };

    public:
        static map_type &getRegistry()
        {
            return registry_;
        }

        static std::vector<std::string> available()
        {
            std::vector<std::string> available;
            for (auto const &values : registry_)
                available.push_back(values.first);
            return available;
        }

        static std::shared_ptr<Initializer<TState, TAction>> make(std::string name)
        {
            typename map_type::iterator it = getRegistry().find(name);
            if (it == getRegistry().end())
                return 0;
            return it->second();
        }

        template <template <typename TS, typename TA> class TValueFunction>
        static void addToRegistry(std::string name)
        {
            if (getRegistry().find(name) == getRegistry().end())
            {
                getRegistry().insert(std::make_pair(name, &createInstance<TState, TAction, TValueFunction>));
            }
            else
            {
                sdm::exception::Exception("Key '" + name + "' already used in ValueFunction Registry.");
            }
        }
    };

    template <typename TState, typename TAction>
    std::shared_ptr<Initializer<TState, TAction>> makeValueFunction(std::string init_name)
    {
        return ValueFunctionFactory<TState, TAction>::make(init_name);
    }

} // namespace sdm
