#include <sdm/core/space/function_space.hpp>
#include <sdm/utils/struct/iterator/function_iterator.hpp>

namespace sdm
{
    template <typename TFunction>
    // FunctionSpace<TFunction>::FunctionSpace(const std::shared_ptr<Space> &input_sp, const std::shared_ptr<Space> &output_sp, bool store_functions) : input_space_(input_sp), output_space_({output_sp})
    FunctionSpace<TFunction>::FunctionSpace(const std::shared_ptr<Space> &input_sp, const std::shared_ptr<Space> &output_sp, bool store_functions, const std::shared_ptr<Space>& action_space) : action_space(action_space)
    {
        this->input_space_ = input_sp;
        this->output_space_ = {output_sp};
        
        // Set the 'store_items_' parameter to true or false
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction>
    FunctionSpace<TFunction>::FunctionSpace(std::vector<input_type> possible_inputs, std::vector<output_type> possible_outputs, bool store_functions, const std::shared_ptr<Space>& action_space): action_space(action_space)
    {
        this->input_space_ = std::make_shared<DiscreteSpace>(possible_inputs);
        this->output_space_.push_back(std::make_shared<DiscreteSpace>(possible_outputs));
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction>
    FunctionSpace<TFunction>::FunctionSpace(const std::shared_ptr<Space> &input_space, const std::vector<std::shared_ptr<Space>> &output_spaces, bool store_functions, const std::shared_ptr<Space>& action_space) : input_space_(input_space), output_space_(output_spaces), action_space(action_space)
    {
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction>
    FunctionSpace<TFunction>::FunctionSpace(std::vector<input_type> possible_inputs, std::vector<std::vector<output_type>> possible_outputs, bool store_functions, const std::shared_ptr<Space>& action_space) : action_space(action_space)
    {
        assert(possible_inputs.size() == possible_outputs.size());

        this->input_space_ = std::make_shared<DiscreteSpace>(possible_inputs);
        for (const auto &v : possible_outputs)
        {
            this->output_space_.push_back(std::make_shared<DiscreteSpace>(v));
        }
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction>
    typename FunctionSpace<TFunction>::iterator_type FunctionSpace<TFunction>::begin()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateItems();
            }
            return DiscreteSpace::begin();
        }
        else
        {
            std::vector<std::shared_ptr<ItemIterator>> out_begin_iterators, out_end_iterators;
            std::vector<std::shared_ptr<Item>> list_input;
            number i = 0;

            for (const auto &input : *this->input_space_)
            {
                list_input.push_back(input);
                out_begin_iterators.push_back(this->output_space_[(this->output_space_.size() == 1) ? 0 : i]->begin());
                out_end_iterators.push_back(this->output_space_[(this->output_space_.size() == 1) ? 0 : i]->end());
                i++;
            }
            return  std::make_shared<sdm::iterator::FunctionIterator<TFunction>>(list_input, out_begin_iterators, out_end_iterators, this->action_space);
        }
    }

    template <typename TFunction>
    typename FunctionSpace<TFunction>::iterator_type FunctionSpace<TFunction>::end()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateItems();
            }
            return DiscreteSpace::end();
        }
        else
        {
            return std::make_shared<sdm::iterator::FunctionIterator<TFunction>>();
        }
    }

} // namespace sdm
