
/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/types.hpp>
#include <sdm/utils/decision_rules/joint.hpp>

namespace sdm
{

  template <typename item, int instance>
  joint<item, instance>::joint(agent num_agents, const std::vector<item> &cdecisions) : cdecisions(cdecisions), num_agents(num_agents)
  {
    joint_item_bimap.insert(jitem2index(this, counter));
    counter++;
  }

  template <typename item, int instance>
  joint<item, instance>::~joint()
  {
    typename bimap::const_iterator iter, iend;
    for (iter = joint_item_bimap.begin(), iend = joint_item_bimap.end(); iter != iend; ++iter)
    {
      delete iter->left;
    }
  }

  template <typename item, int instance>
  item joint<item, instance>::getJointItemIdx(std::vector<item> const &v)
  {
    typename bimap::const_iterator iter;

    //Warning: the following assumes that the operator == is defined over elements of type "item"
    for (iter = joint_item_bimap.begin(); iter != joint_item_bimap.end(); ++iter)
    {
      if (iter->left->cdecisions == v)
      {
        return iter->right;
      }
    }

    return iter->right;
  }

  template <typename item, int instance>
  item joint<item, instance>::getIndividualItem(agent ag)
  {
    assert(this->cdecisions.size() == this->num_agents);
    return this->cdecisions[ag];
  }

  template <typename item, int instance>
  item joint<item, instance>::getJointItemIdx(joint<item, instance> *j)
  {
    assert(joint_item_bimap.left.find(j) != joint_item_bimap.left.end());
    return joint_item_bimap.left.at(j);
  }

  template <typename item, int instance>
  joint<item, instance> *joint<item, instance>::getJointItem(item i)
  {
    assert(joint_item_bimap.right.find(i) != joint_item_bimap.right.end());
    return joint_item_bimap.right.at(i);
  }

  template <typename item, int instance>
  number joint<item, instance>::counter = 0;

  template <typename item, int instance>
  typename joint<item, instance>::bimap joint<item, instance>::joint_item_bimap = {};

  template class joint<number, 2>;
  template class joint<action, 0>;
  template class joint<observation, 1>;
} // namespace sdm
