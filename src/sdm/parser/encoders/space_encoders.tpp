#include <sdm/parser/encoders/space_encoders.hpp>

namespace sdm
{
    namespace ast
    {
        template <typename TItem>
        std::shared_ptr<DiscreteSpace> discrete_space_encoder<TItem>::operator()(number ag) const
        {
            std::vector<std::shared_ptr<Item>> values;
            for (int i = 0; i < ag; i++)
            {
                values.push_back(std::make_shared<TItem>(std::to_string(i)));
            }
            return std::make_shared<DiscreteSpace>(values);
        }

        template <typename TItem>
        std::shared_ptr<DiscreteSpace> discrete_space_encoder<TItem>::operator()(const std::vector<std::string> &ags) const
        {
            std::vector<std::shared_ptr<Item>> values;
            for (const auto &str : ags)
            {
                values.push_back(std::make_shared<TItem>(str));
            }
            return std::make_shared<DiscreteSpace>(values);
        }

        template <typename TItem>
        std::shared_ptr<MultiDiscreteSpace> multi_discrete_space_encoder<TItem>::operator()(const std::vector<number> &dim_spaces) const
        {
            std::vector<std::shared_ptr<Space>> list_spaces;
            discrete_space_encoder<TItem> ds_encoder;
            for (auto &dim : dim_spaces)
            {
                list_spaces.push_back(ds_encoder(dim));
            }
            return std::make_shared<MultiDiscreteSpace>(list_spaces);
        }

        template <typename TItem>
        std::shared_ptr<MultiDiscreteSpace> multi_discrete_space_encoder<TItem>::operator()(const std::vector<std::vector<std::string>> &all_list_names) const
        {
            std::vector<std::shared_ptr<Space>> list_spaces;
            discrete_space_encoder<TItem> ds_encoder;
            for (auto &list_names : all_list_names)
            {
                list_spaces.push_back(ds_encoder(list_names));
            }
            return std::make_shared<MultiDiscreteSpace>(list_spaces);
        }
    }
}