#include <sdm/core/space/function_space.hpp>

namespace sdm
{
    template <typename TFunction>
    FunctionSpace<TFunction>::FunctionSpace(input_space input_sp, output_space output_sp) : input_space_(input_sp), output_space_({output_sp})
    {
    }

    template <typename TFunction>
    FunctionSpace<TFunction>::FunctionSpace(std::vector<input_type> possible_inputs, std::vector<output_type> possible_outputs) : input_space_(possible_inputs)
    {
        this->output_space_.push_back(output_space(possible_outputs));
    }

    template <typename TFunction>
    FunctionSpace<TFunction>::FunctionSpace(input_space input_space, std::vector<output_space> output_spaces) : input_space_(input_space), output_space_(output_spaces)
    {
        assert(input_sp.getNumElements() == output_sps.size());
    }

    template <typename TFunction>
    FunctionSpace<TFunction>::FunctionSpace(std::vector<input_type> possible_inputs, std::vector<std::vector<output_type>> possible_outputs) : input_space_(possible_inputs)
    {
        assert(possible_inputs.size() == possible_outputs.size());
        for (const auto &v : possible_outputs)
        {
            this->output_space_.push_back(output_space(v));
        }
    }

    template <typename TFunction>
    std::vector<TFunction> FunctionSpace<TFunction>::getAll()
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


    template <typename TFunction>
    FunctionSpace<TFunction>::iterator_type FunctionSpace<TFunction>::begin()
    {
        return std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list_items_.begin())>>(this->list_items_.begin());
    }

    template <typename TFunction>
    FunctionSpace<TFunction>::iterator_type FunctionSpace<TFunction>::end()
    {
        return std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list_items_.end())>>(this->list_items_.end());
    }


} // namespace sdm
