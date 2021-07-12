#include <sdm/utils/linear_programming/lp_problem.hpp>

namespace sdm
{
    LPBase::LPBase() {}

    LPBase::LPBase(const std::shared_ptr<SolvableByHSVI>&world) : world_(world){}

    LPBase::~LPBase() {}

    Pair<std::shared_ptr<Action>,double> LPBase::createLP(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, number t)
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

            // std::cout<<"createVariables"<<std::endl;

            this->createVariables(vf,state, env, var,index, t);

            // std::cout<<"createObjectiveFunction"<<std::endl;

            this->createObjectiveFunction(vf, state, var, obj, t);

            // std::cout<<"createConstraints"<<std::endl;

            index = 0;
            this->createConstraints(vf, state, env,model, con, var, index, t);

            // std::cout<<"End Create"<<std::endl;

            ///////  END CORE  CPLEX Code ///////
            model.add(obj);
            model.add(con);
            IloCplex cplex(model);
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());

            cplex.exportModel("lb_bellman_op.lp");

            // Optimize the problem and obtain solution
            if (!cplex.solve())
            {
                env.error() << "Failed to optimize MILP" << std::endl;
            }
            else
            {
                value = cplex.getObjValue();
                action = this->getVariableResult(vf,state, cplex,var,t);

                // std::cout<<"action "<<action->str()<<std::endl;
                // if(std::abs(value - vf->template backup<double>(state,action,t)) >0.01)
                // {
                //     std::cout<<"Problem, value "<<value<<" backup value "<<vf->template backup<double>(state,action,t)<<std::endl;
                //     std::cout<<"State "<<state->str()<<std::endl;
                //     std::cout<<"action "<<action->str()<<std::endl;
                //     system("cat lb_bellman_op.lp");

                //     exit(-1);
                // }

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

        return std::make_pair(action,value);
    }
}