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
        FunctionSpace(input_space input_sp, output_space output_sp) : input_space_(input_sp), output_space_({output_sp})
        {
        }

        /**
         * @brief Construct a new Function Space object
         * 
         * @param possible_inputs possible inputs
         * @param possible_outputs possible ouputs 
         */
        FunctionSpace(std::vector<input_type> possible_inputs, std::vector<output_type> possible_outputs) : input_space_(possible_inputs)
        {
            this->output_space_.push_back(output_space(possible_outputs));
        }

        /**
         * @brief Construct a new Function Space object
         * 
         * @param input_space
         * @param output_sps output spaces, one for each input value (requirements : input_space.size() == output_spaces.size() or output_spaces.size()==1). 
         */
        FunctionSpace(input_space input_space, std::vector<output_space> output_spaces) : input_space_(input_space), output_space_(output_spaces)
        {
            assert(input_sp.getNumElements() == output_sps.size());
        }

        FunctionSpace(std::vector<input_type> possible_inputs, std::vector<std::vector<output_type>> possible_outputs) : input_space_(possible_inputs)
        {
            assert(possible_inputs.size() == possible_outputs.size());
            for (const auto &v : possible_outputs)
            {
                this->output_space_.push_back(output_space(v));
            }
        }

        /**
         * @brief Get all the possible function in this space.
         * 
         * @return the list of all possible functions
         */
        std::vector<TFunction> getAll()
        {
            assert(this->output_space_.size() > 0);
            if (this->all_items_.empty())
            {
                // Build appropriate input for Variation (vector of vector of output)
                std::vector<std::vector<output_type>> tmp;
                for (std::size_t i = 0; i < input_space_.getAll().size(); i++)
                {
                    if (this->output_space_.size() == 1)
                    {
                        tmp.push_back(this->output_space_[0].getAll());
                    }
                    else
                    {
                        tmp.push_back(this->output_space_[i].getAll());
                    }
                }

                // Generate all possible functions
                Variations<TFunction> funct_generator(input_space_.getAll(), tmp);
                number idx = 0;
                for (auto it = funct_generator.begin(); it != funct_generator.end(); it = funct_generator.next())
                {
                    this->all_items_.insert(funct_bimap_value(idx, *it));
                    idx++;
                    this->list_items_.push_back(*it);
                }
                this->num_items_ = this->all_items_.size();
            }
            return DiscreteSpace<TFunction>::getAll();
        }
    };
} // namespace sdm
