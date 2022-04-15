/**
 * @file function_space.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File for discrete function space
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/variations.hpp>

namespace sdm
{
    /**
     * @brief The class for function spaces. This is helpfull in case we need to enumerate all possible functions (only usable when input space and output space are DiscreteSpace).
     * 
     * @tparam TFunction The type of function to generate.
     */
    template <typename TFunction>
    class FunctionSpace : public DiscreteSpace<TFunction>
    {
    protected:
        using value_type = TFunction;
        using input_type = typename TFunction::input_type;
        using output_type = typename TFunction::output_type;

        /**
         * @brief The input space 
         */
        std::shared_ptr<BaseSpace<input_type>> input_space_;

        /**
         * @brief The vector of output spaces (possibly one output space for each input space). In case output space are similar for all input, use the adequate constructor.
         */
        std::vector<std::shared_ptr<BaseSpace<output_type>>> output_space_;

        std::shared_ptr<BaseSpace<output_type>> action_space;

    public:
        using iterator_type = DiscreteSpace::iterator_type;

        FunctionSpace(const std::shared_ptr<BaseSpace<input_type>> &input_space, const std::shared_ptr<BaseSpace<output_type>> &output_space, bool store_functions = false, const std::shared_ptr<BaseSpace<output_type>>& action_space = nullptr);

        /**
         * @brief Construct a new Function Space object
         * 
         * @param possible_inputs possible inputs
         * @param possible_outputs possible ouputs 
         */
        FunctionSpace(std::vector<input_type> possible_inputs, std::vector<output_type> possible_outputs, bool store_functions = false, const std::shared_ptr<Space>& action_space = nullptr);

        /**
         * @brief Construct a new Function Space object
         * 
         * @param input_space
         * @param output_sps output spaces, one for each input value (requirements : input_space->size() == output_spaces.size() or output_spaces.size()==1). 
         */
        FunctionSpace(const std::shared_ptr<Space> &input_space, const std::vector<std::shared_ptr<Space>> &output_spaces, bool store_functions = false, const std::shared_ptr<Space>& action_space = nullptr);

        FunctionSpace(std::vector<input_type> possible_inputs, std::vector<std::vector<output_type>> possible_outputs, bool store_functions = false, const std::shared_ptr<Space>& action_space = nullptr);

        iterator_type begin();
        iterator_type end();

    };
} // namespace sdm
#include <sdm/core/space/function_space.tpp>