#include <sdm/utils/linear_programming/lp_problem.hpp>

namespace sdm
{
    LPBase::LPBase() {}

    LPBase::LPBase(const std::shared_ptr<SolvableByHSVI> &world) : world_(world) {}

    LPBase::~LPBase() {}

    Pair<std::shared_ptr<Action>, double> LPBase::createLP(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        number index = 0;

        //<! greedy decision, initialization
        std::shared_ptr<Action> action;
        double value;

        IloEnv env;
        try
        {
            IloModel model(env);

            // Init the model
            IloRangeArray con(env);
            IloNumVarArray var(env);

            IloObjective obj = IloMaximize(env);

            ///////  BEGIN CORE CPLEX Code  ///////

            // Create all Variable of the LP problem
            this->createVariables(vf, state, env, var, index, t);

            // Create the objective function of the LP problem
            this->createObjectiveFunction(vf, state, var, obj, t);

            index = 0;

            // Create all Constraints of the LP problem
            this->createConstraints(vf, state, env, model, con, var, index, t);

            ///////  END CORE  CPLEX Code ///////
            model.add(obj);
            model.add(con);
            IloCplex cplex(model);
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());

            // Optimize the problem and obtain solution
            if (!cplex.solve())
            {
                env.error() << "Failed to optimize MILP" << std::endl;
                cplex.exportModel("lb_bellman_op.lp");
                system("cat lb_bellman_op.lp");

            }
            else
            {
                value = cplex.getObjValue();
                action = this->getVariableResult(vf, state, cplex, var, t);
            }
        }
        catch (IloException &e)
        {
            std::cerr << "Concert exception caught: " << e << std::endl;
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
        }

        env.end();

        return std::make_pair(action, value);
    }
}