/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/utils/decision_rules/variations.hpp>
#include <sdm/types.hpp>

namespace sdm
{
    //! \class  Space  space.hpp
    template <typename TFunction>
    class FunctionSpace : DiscreteSpace<TFunction>
    {
    protected:
        using value_type = TFunction;
        using input_type = typename TFunction::input_type;
        using output_type = typename TFunction::output_type;

        using input_space = DiscreteSpace<input_type>;
        using output_space = DiscreteSpace<output_type>;

        input_space input_space_;
        std::vector<output_space> output_space_;

    public:
        FunctionSpace(input_space input_sp, output_space output_sp) : input_space_(input_sp), output_space_({output_sp})
        {
        }

        FunctionSpace(std::vector<input_type> possible_inputs, std::vector<output_type> possible_outputs) : input_space_(possible_inputs)
        {
            this->output_space_.push_back(output_space(possible_outputs));
        }

        FunctionSpace(input_space input_sp, std::vector<output_space> output_sps) : input_space_(input_sp), output_space_(output_sps)
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

        std::vector<TFunction> getAll()
        {
            assert(this->output_space_.size() > 0);
            if (this->all_items_.empty())
            {
                std::vector<std::vector<output_type>> tmp;
                for (int i = 0; i < input_space_.getAll().size(); i++)
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
                Variations<TFunction> funct_generator(input_space_.getAll(), tmp);
                for (auto it = funct_generator.begin(); it != funct_generator.end(); it = funct_generator.next())
                {
                    this->all_items_.push_back(*it);
                }
                this->num_elements_ = this->all_items_.size();
            }
            return this->all_items_;
        }
    };
} // namespace sdm
