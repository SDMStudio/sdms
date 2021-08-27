/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <iostream>
#include <algorithm>

#include <sdm/core/variations.hpp>

namespace sdm
{
    template <typename TFunction>
    Variations<TFunction>::Variations() {}

    template <typename TFunction>
    Variations<TFunction>::Variations(const std::vector<std::vector<TItem>> &possible_values) : Variations({}, possible_values)
    {
    }

    template <typename TFunction>
    Variations<TFunction>::Variations(const std::vector<TKey> &possible_keys, const std::vector<std::vector<TItem>> &possible_values) : p_keys_(possible_keys), p_values_(possible_values)
    {
        this->setVariation(possible_keys, possible_values);
    }

    template <typename TFunction>
    void Variations<TFunction>::setVariation(const std::vector<TKey> &possible_keys, const std::vector<std::vector<TItem>> &possible_values)
    {
        this->p_keys_ = possible_keys;
        this->p_values_ = possible_values;
        this->dimension = this->p_values_.size();

        for (std::size_t i = 0; i < this->p_values_.size(); i++)
        {
            this->current.push_back(this->p_values_[i].begin());
        }

        this->vin = this->make_output();
        this->vout = nullptr;
    }

    template <typename TFunction>
    std::shared_ptr<TFunction> Variations<TFunction>::make_output()
    {
        std::vector<TItem> values;
        for (auto it : this->current)
        {
            values.push_back(*it);
        }
        return std::shared_ptr<TFunction>(new TFunction(this->p_keys_, values));
    }

    template <typename TFunction>
    std::shared_ptr<TFunction> Variations<TFunction>::begin()
    {
        return this->vin;
    }

    template <typename TFunction>
    std::shared_ptr<TFunction> Variations<TFunction>::end()
    {
        return nullptr;
    }

    template <typename TFunction>
    std::shared_ptr<TFunction> Variations<TFunction>::next()
    {
        bool end = false;
        for (int i = this->dimension - 1; i >= 0; i--)
        {
            if (this->current[i] != this->p_values_[i].end() - 1)
            {
                this->current[i]++;
                break;
            }
            else
            {
                if (i == 0)
                {
                    end = true;
                    return this->end();
                }
                else
                {
                    this->current[i] = this->p_values_[i].begin();
                }
            }
        }
        return this->make_output();
    }
} // namespace sdm
