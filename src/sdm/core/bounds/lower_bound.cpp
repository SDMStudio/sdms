#include <sdm/core/bounds/lower_bound.hpp>

namespace sdm
{

    template <typename TState>
    LowerBound<TState>::LowerBound(std::shared_ptr<POSG> problem) : problem_(problem) {}

    template <typename TState>
    void LowerBound<TState>::setValueAt(TState state, Vector value)
    {
        this->vector_set.emplace(state, value);
    }

    template <typename TState>
    void LowerBound<TState>::updateValueAt(TState &s)
    {
        Vector new_hyperplan = this->backup();
        this->setValueAt(s, new_hyperplan);
        this->prune();
    }

    template <typename TState>
    double LowerBound<TState>::getValueAt(TState x)
    {
    }

    template <typename TState>
    number LowerBound<TState>::size()
    {
        return this->vector_set.size()
    }

    template <typename TState>
    void LowerBound<TState>::prune()
    {
        this->lark_prune();
    }

    template <typename TState>
    std::pair<std::shared_ptr<Vector>, double> LowerBound<TState>::checkDominance(const std::shared_ptr<Vector> &alpha, const std::unordered_set<std::shared_ptr<Vector>> &vectors)
    {
        double value = 0;
        number index = 0, c = 0;
        std::shared_ptr<Vector> f_state;

        IloEnv env;
        try
        {
            IloModel model(env);
            IloRangeArray con(env);
            IloNumVarArray var(env);
            IloObjective obj = IloMaximize(env);

            common::clearVariables();
            // build variables upper, lower, frequency_state
            var.add(IloNumVar(env, -IloInfinity, +IloInfinity, "value"));
            common::setNumber("value", index++);
            for (state x = 0; x < common::model->getNumStates(); ++x)
            {
                stringstream name;
                name << "x" << x;
                common::setNumber(name.str(), index++);
                var.add(IloNumVar(env, 0.0, 1.0, name.str().c_str()));
            }

            // build objective function
            obj.setLinearCoef(var[common::getNumber("value")], +1.0);

            // build constraints
            for (const auto &entry : vectors)
            {
                // constraint  value <= <current - competitor, s> for all competitor != current
                con.add(IloRange(env, -IloInfinity, 0.0));
                con[c].setLinearCoef(var[common::getNumber("value")], +1.0);
                for (state x = 0; x < common::model->getNumStates(); ++x)
                {
                    stringstream name;
                    name << "x" << x;
                    con[c].setLinearCoef(var[common::getNumber(name.str())], (*entry)[x] - (*alpha)[x]);
                }
                c++;
            }

            // constraint  |s|_1 = 1
            con.add(IloRange(env, 1.0, 1.0));
            for (state x = 0; x < common::model->getNumStates(); ++x)
            {
                stringstream name;
                name << "x" << x;
                con[c].setLinearCoef(var[common::getNumber(name.str())], +1.0);
            }

            model.add(obj);
            model.add(con);
            IloCplex cplex(model);

            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
            cplex.exportModel("vector-pruning.lp");

            if (!cplex.solve())
            {
                env.error() << "Failed to optimize vector-set lp" << std::endl;
                system("cat vector-pruning.lp");
                throw(-1);
            }

            value = cplex.getObjValue();

            f_state = std::make_shared<Vector>(common::model->getNumStates());
            f_state->init(0.0);
            for (state x = 0; x < common::model->getNumStates(); ++x)
            {
                stringstream name;
                name << "x" << x;
                (*f_state)[x] = cplex.getValue(var[common::getNumber(name.str())]);
            }
        }
        catch (IloException &e)
        {
            std::cerr << "Concert exception caught: " << e << std::endl;
        }
        catch (const std::exception &exc)
        {
            std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
        }
        env.end();

        return std::make_pair(f_state, value);
    }

    template <typename TState>
    void LowerBound<TState>::lark_prune()
    {
        number i, max_i;
        double epsilon = 0.0, max_value, value;
        std::vector<std::shared_ptr<Vector>> deltaContainer;
        std::unordered_set<std::shared_ptr<Vector>> newContainer;
        std::unordered_map<std::shared_ptr<Vector>, std::shared_ptr<Vector>> deltaContainerMap;

        for (auto &entry : this->vector_set)
        {
            deltaContainer.push_back(entry.second);
            deltaContainerMap.emplace(entry.second, entry.first);
        }

        while (!deltaContainer.empty())
        {
            auto res = newContainer.empty() ? std::make_pair(deltaContainerMap.at(deltaContainer[0]), 1.0) : this->checkDominance(deltaContainer[0], newContainer);
            if (res.second > epsilon)
            {
                max_i = 0;
                max_value = std::numeric_limits<double>::min();
                for (i = 0; i < deltaContainer.size(); ++i)
                {
                    // std::cout << "i=" << i << "\tmax_i=" << max_i << std::endl;
                    if (max_value < (value = (*deltaContainer[i]) ^ (*res.first)))
                    {
                        max_i = i;
                        max_value = value;
                    }
                }
                // std::cout << deltaContainer[max_i] << std::endl;
                // std::cout <<  "\t" << *deltaContainer[max_i] << std::endl;
                newContainer.insert(deltaContainer[max_i]);
                deltaContainer.erase(deltaContainer.begin() + max_i);
            }

            else
            {
                deltaContainer.erase(deltaContainer.begin() + 0);
            }
        }

        for (auto it = this->vector_set.begin(); it != this->vector_set.end(); ++it)
        {
            if (newContainer.find(it->second) == newContainer.end())
                this->vector_set.erase(it);
        }
    }
} // namespace sdm