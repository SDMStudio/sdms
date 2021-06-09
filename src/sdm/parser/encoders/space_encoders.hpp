#pragma once

#include <sdm/parser/ast.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{
    namespace ast
    {
        /**
         * @brief encodes the input into a discrete space class
         */
        struct discrete_space_encoder : boost::static_visitor<std::shared_ptr<DiscreteSpace>>
        {
            std::shared_ptr<DiscreteSpace> operator()(number ag) const;
            std::shared_ptr<DiscreteSpace> operator()(const std::vector<std::string> &ags) const;
        };

        /**
         * @brief encodes the input into a multi discrete space class
         */
        struct multi_discrete_space_encoder : boost::static_visitor<std::shared_ptr<MultiDiscreteSpace>>
        {
            std::shared_ptr<MultiDiscreteSpace> operator()(const std::vector<number> &dim_spaces) const;
            std::shared_ptr<MultiDiscreteSpace> operator()(const std::vector<std::vector<std::string>> &all_list_names) const;
        };
    } // namespace ast

} // namespace sdm
