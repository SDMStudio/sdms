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

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/variations.hpp>
#include <sdm/types.hpp>

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

        typedef boost::bimaps::bimap<number, TFunction> funct_bimap;
        typedef typename funct_bimap::value_type funct_bimap_value;

        using value_type = TFunction;
        using input_type = typename TFunction::input_type;
        using output_type = typename TFunction::output_type;

        using input_space = DiscreteSpace<input_type>;
        using output_space = DiscreteSpace<output_type>;

        /**
         * @brief The input space 
         */
        input_space input_space_;

        /**
         * @brief The vector of output spaces (possibly one output space for each input space). In case output space are similar for all input, use the adequate constructor.
         */
        std::vector<output_space> output_space_;

    public:
        FunctionSpace(input_space input_sp, output_space output_sp);
        /**
         * @brief Construct a new Function Space object
         * 
         * @param possible_inputs possible inputs
         * @param possible_outputs possible ouputs 
         */
        FunctionSpace(std::vector<input_type> possible_inputs, std::vector<output_type> possible_outputs);

        /**
         * @brief Construct a new Function Space object
         * 
         * @param input_space
         * @param output_sps output spaces, one for each input value (requirements : input_space.size() == output_spaces.size() or output_spaces.size()==1). 
         */
        FunctionSpace(input_space input_space, std::vector<output_space> output_spaces);

        FunctionSpace(std::vector<input_type> possible_inputs, std::vector<std::vector<output_type>> possible_outputs);

        /**
         * @brief Get all the possible function in this space.
         * 
         * @return the list of all possible functions
         */
        std::vector<TFunction> getAll();
    };
} // namespace sdm
