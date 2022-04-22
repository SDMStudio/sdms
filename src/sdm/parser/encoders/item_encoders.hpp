#pragma once

#include <sdm/parser/encoders/space_encoders.hpp>

namespace sdm
{

    namespace ast
    {

        //! \struct item_encoder
        //! \brief encodes the input into a item index (string))
        template <typename TItem>
        struct item_encoder : boost::static_visitor<std::shared_ptr<typename TItem::base>>
        {
            std::shared_ptr<MultiDiscretePtrSpace<typename TItem::base>> md_space_;
            number ag;

            item_encoder(const std::shared_ptr<MultiDiscretePtrSpace<typename TItem::base>> &md_space, number ag) : boost::static_visitor<std::shared_ptr<typename TItem::base>>()
            {
                this->ag = ag;
                this->md_space_ = md_space;
            }

            std::shared_ptr<typename TItem::base> operator()(number a) const
            {
                return std::static_pointer_cast<DiscretePtrSpace<typename TItem::base>>(this->md_space_->getSpace(this->ag))->getItem(a);
            }

            std::shared_ptr<typename TItem::base> operator()(const std::string &a_str) const
            {
                return std::static_pointer_cast<DiscretePtrSpace<typename TItem::base>>(this->md_space_->getSpace(this->ag))->getItemAddress(TItem(a_str));
            }
        };

        struct str_visitor
        {
            using result_type = std::string;

            result_type operator()(number v) const { return std::to_string(v); }
            result_type operator()(const std::string &v) { return v; }
        };

        //! \struct joint_item_encoder
        //! \brief encodes the input into a joint element (vector of number)
        template <typename TItem>
        class joint_item_encoder
        {
        protected:
            std::shared_ptr<MultiDiscretePtrSpace<typename TItem::base>> joint_item_space_;
            std::shared_ptr<DiscretePtrSpace<Item>> ag_space_;

        public:
            joint_item_encoder(const std::shared_ptr<MultiDiscretePtrSpace<typename TItem::base>> &joint_item_space, const std::shared_ptr<DiscretePtrSpace<Item>> &ag_space)
            {
                this->ag_space_ = ag_space;
                this->joint_item_space_ = joint_item_space;
            }

            std::vector<std::shared_ptr<typename TItem::base>> encode(const std::vector<identifier_t> &list_items) const
            {
                std::vector<std::shared_ptr<typename TItem::base>> a_vec;
                str_visitor str_vis;

                if (list_items.size() == 1 && boost::apply_visitor(str_vis, list_items[0]) == "*")
                {
                    for (const auto &jitem : *this->joint_item_space_)
                    {
                        a_vec.push_back(jitem);
                    }
                }
                else
                {
                    auto joint_item = std::make_shared<Joint<std::shared_ptr<typename TItem::base>>>();
                    for (number ag = 0; ag < this->ag_space_->getNumItems(); ++ag)
                    {
                        item_encoder<TItem> a_encoder(this->joint_item_space_, ag);
                        std::shared_ptr<typename TItem::base> a_ = boost::apply_visitor(a_encoder, list_items[ag]);
                        joint_item->push_back(a_);
                    }
                    a_vec.push_back(this->joint_item_space_->getItemAddress(*joint_item));
                }
                return a_vec;
            }
        };

        /** 
         * @brief encodes the input into a vector of number (vector of states)
         * "*" -> [0,1,2,3,4,...,n]
         * "s0" -> [0]
         * 0 -> [0]
         *
         */
        template <typename TItem>
        struct state_encoder : boost::static_visitor<std::vector<std::shared_ptr<typename TItem::base>>>
        {

            std::shared_ptr<DiscretePtrSpace<typename TItem::base>> space_;

            state_encoder(const std::shared_ptr<DiscretePtrSpace<typename TItem::base>> &space) : boost::static_visitor<std::vector<std::shared_ptr<typename TItem::base>>>()
            {
                this->space_ = space;
            }

            std::vector<std::shared_ptr<typename TItem::base>> operator()(number s) const
            {
                return {this->space_->getItem(s)};
            }

            std::vector<std::shared_ptr<typename TItem::base>> operator()(const std::string &s_str) const
            {
                std::vector<std::shared_ptr<typename TItem::base>> st_ptr;
                if (s_str == "*")
                {
                    for (number s = 0; s < this->space_->getNumItems(); ++s)
                        st_ptr.push_back(this->space_->getItem(s));
                }
                else
                    st_ptr.push_back(this->space_->getItemAddress(TItem(s_str)));

                return st_ptr;
            }
        };

    } // namespace ast

} // namespace sdm
