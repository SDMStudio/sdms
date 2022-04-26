#include <sdm/core/space/function_space.hpp>
#include <sdm/core/space/iterator/function_iterator.hpp>

namespace sdm
{
    template <typename TFunction, typename TBase>
    FunctionSpace<TFunction, TBase>::FunctionSpace(const std::shared_ptr<TInputSpace> &input_sp, const std::shared_ptr<TOutputSpace> &output_sp, bool store_functions, const std::shared_ptr<TOutputSpace> &action_space) : action_space(action_space)
    {
        this->input_space = input_sp;
        this->output_space = {output_sp};

        // Set the 'store_items_' parameter to true or false
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction, typename TBase>
    FunctionSpace<TFunction, TBase>::FunctionSpace(const std::shared_ptr<TOutputSpace> &output_sp, bool store_functions, const std::shared_ptr<TOutputSpace> &action_space) : action_space(action_space)
    {
        this->input_space = std::make_shared<DiscreteSpace<input_type>>(std::vector<input_type>({nullptr}));
        this->output_space = {output_sp};

        // Set the 'store_items_' parameter to true or false
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction, typename TBase>
    FunctionSpace<TFunction, TBase>::FunctionSpace(std::vector<input_type> possible_inputs, std::vector<output_type> possible_outputs, bool store_functions, const std::shared_ptr<Space> &action_space) : action_space(action_space)
    {
        this->input_space = std::make_shared<DiscreteSpace<input_type>>(possible_inputs);
        this->output_space.push_back(std::make_shared<DiscreteSpace<output_type>>(possible_outputs));
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction, typename TBase>
    FunctionSpace<TFunction, TBase>::FunctionSpace(const std::shared_ptr<TInputSpace> &input_space, const std::vector<std::shared_ptr<TOutputSpace>> &output_spaces, bool store_functions, const std::shared_ptr<TOutputSpace> &action_space) : input_space(input_space), output_space(output_spaces), action_space(action_space)
    {
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction, typename TBase>
    FunctionSpace<TFunction, TBase>::FunctionSpace(std::vector<input_type> possible_inputs, std::vector<std::vector<output_type>> possible_outputs, bool store_functions, const std::shared_ptr<TOutputSpace> &action_space) : action_space(action_space)
    {
        assert(possible_inputs.size() == possible_outputs.size());

        this->input_space = std::make_shared<DiscreteSpace<input_type>>(possible_inputs);
        for (const auto &v : possible_outputs)
        {
            this->output_space.push_back(std::make_shared<DiscreteSpace<output_type>>(v));
        }
        this->storeItems(store_functions);
        if (this->isStoringItems())
        {
            this->generateItems();
        }
    }

    template <typename TFunction, typename TBase>
    typename FunctionSpace<TFunction, TBase>::iterator_type FunctionSpace<TFunction, TBase>::begin()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateItems();
            }
            return DiscreteSpace<std::shared_ptr<TBase>>::begin();
        }
        else
        {
            std::vector<std::shared_ptr<Iterator<output_type>>> out_begin_iterators, out_end_iterators;
            std::vector<input_type> list_input;
            number i = 0;
            auto end_iter = this->input_space->end();
            for (auto iter = this->input_space->begin(); !iter->equal(end_iter); iter = iter->next())
            {
                auto input = iter->getCurrent();
                list_input.push_back(input);
                out_begin_iterators.push_back(this->output_space[(this->output_space.size() == 1) ? 0 : i]->begin());
                out_end_iterators.push_back(this->output_space[(this->output_space.size() == 1) ? 0 : i]->end());
                i++;
            }
            return std::make_shared<sdm::iterator::FunctionIterator<TFunction, TBase>>(list_input, out_begin_iterators, out_end_iterators, this->action_space);
        }
    }

    template <typename TFunction, typename TBase>
    typename FunctionSpace<TFunction, TBase>::iterator_type FunctionSpace<TFunction, TBase>::end()
    {
        if (this->isStoringItems())
        {
            if (!this->isGenerated())
            {
                this->generateItems();
            }
            return DiscreteSpace<std::shared_ptr<TBase>>::end();
        }
        else
        {
            return std::make_shared<sdm::iterator::FunctionIterator<TFunction, TBase>>();
        }
    }

} // namespace sdm
