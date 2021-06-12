// #include <sdm/config.hpp>

// namespace sdm
// {
//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     TabularValueFunction<TItem, TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, std::shared_ptr<Initializer> initializer)
//         : ValueFunction(problem, horizon), initializer_(initializer)
//     {
//         this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     TabularValueFunction<TItem, TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, double default_value)
//         : TabularValueFunction(problem, horizon, std::make_shared<ValueInitializer>(default_value))
//     {
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     void TabularValueFunction<TItem, TBackupOperator, TStruct>::initialize()
//     {
//         this->initializer_->init(this->getptr());
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     void TabularValueFunction<TItem, TBackupOperator, TStruct>::initialize(double default_value, number t)
//     {
//         this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     double TabularValueFunction<TItem, TBackupOperator, TStruct>::getValueAt(const std::shared_ptr<Item> &state, number t)
//     {
//         if (t < this->getHorizon() && this->init_function_ != nullptr)
//         {
//             if ((this->representation[t].find(*state->to<TItem>()) == this->representation[t].end()))
//             {
//                 double i_value = this->init_function_->operator()(state, t);
//                 this->updateValueAt(state, t, i_value);
//                 return i_value;
//             }
//         }

//         return this->representation[this->isInfiniteHorizon() ? 0 : t].at(*state->to<TItem>());
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     std::shared_ptr<Action> TabularValueFunction<TItem, TBackupOperator, TStruct>::getBestAction(const std::shared_ptr<Item> &state, number t)
//     {
//         std::shared_ptr<Action> best_action;
//         double max = -std::numeric_limits<double>::max(), tmp;

//         for (const auto &action : *this->getWorld()->getActionSpaceAt(state->to<State>(), t))
//         {
//             auto casted_action = action->toAction();
//             if (max < (tmp = this->getQValueAt(state, casted_action, t)))
//             {
//                 best_action = casted_action;
//                 max = tmp;
//             }
//         }

//         return best_action;
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     void TabularValueFunction<TItem, TBackupOperator, TStruct>::updateValueAt(const std::shared_ptr<Item> &state, number t, double target)
//     {
//         this->representation[this->isInfiniteHorizon() ? 0 : t][*state->to<TItem>()] = target;
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     void TabularValueFunction<TItem, TBackupOperator, TStruct>::updateValueAt(const std::shared_ptr<Item> &state, number t)
//     {
//         this->updateValueAt(state, t, this->getBackupOperator().backup(this->getptr(), state->toState(), t));
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     typename TabularValueFunction<TItem, TBackupOperator, TStruct>::backup_operator_type TabularValueFunction<TItem, TBackupOperator, TStruct>::getBackupOperator()
//     {
//         return this->backup_op_;
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     void TabularValueFunction<TItem, TBackupOperator, TStruct>::save(std::string filename)
//     {
//         BoostSerializable<TabularValueFunction<TItem, TBackupOperator, TStruct>>::save(filename);
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     void TabularValueFunction<TItem, TBackupOperator, TStruct>::load(std::string filename)
//     {
//         BoostSerializable<TabularValueFunction<TItem, TBackupOperator, TStruct>>::load(filename);
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     std::string TabularValueFunction<TItem, TBackupOperator, TStruct>::str() const
//     {
//         std::ostringstream res;

//         res << "<tabular_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
//         for (std::size_t i = 0; i < this->representation.size(); i++)
//         {
//             res << "\t<value_function timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
//             for (const auto &pair_state_val : this->representation[i])
//             {
//                 //         // res << tools::addIndent(state_str.str(), 3) << std::endl;
//                 res << "\t\t<value state=\"" << pair_state_val.first << "\" value=\"" << std::setprecision(config::VALUE_DECIMAL_PRINT) << pair_state_val.second << "\" />" << std::endl;
//             }
//             res << "\t</value_function>" << std::endl;
//         }

//         res << "</tabular_value_function>" << std::endl;
//         return res.str();
//     }

//     template <class TItem, class TBackupOperator, template <typename TI, typename TV> class TStruct>
//     std::vector<std::shared_ptr<Item>> TabularValueFunction<TItem, TBackupOperator, TStruct>::getSupport(number t)
//     {
//         std::vector<std::shared_ptr<Item>> indexes;
//         for (const auto &index : this->representation[this->isInfiniteHorizon() ? 0 : t].getIndexes())
//         {
//             indexes.push_back(std::make_shared<TItem>(index));
//         }
//         return indexes;
//     }

// } // namespace sdm