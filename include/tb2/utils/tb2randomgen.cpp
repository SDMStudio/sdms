/*
 * ****** Random WCSP generator *******
 */

#include "tb2randomgen.hpp"
#include "core/tb2constraint.hpp"
#include "core/tb2variable.hpp"
#include "core/tb2enumvar.hpp"

bool naryRandom::connected()
{
    return true;
}

void naryRandom::generateGlobalCtr(vector<int>& indexs, string globalname, Cost costMin, Cost costMax)
{
    int i;
    int arity = indexs.size();
    EnumeratedVariable** scopeVars = new EnumeratedVariable*[arity];
    int* scopeIndexs = new int[arity];
    Cost Top = wcsp.getUb();
    if (costMax < Top)
        Top = ToulBar2::costMultiplier * costMax;

    for (i = 0; i < arity; i++) {
        scopeIndexs[i] = indexs[i];
        scopeVars[i] = (EnumeratedVariable*)wcsp.getVar(indexs[i]);
    }

    random_shuffle(&scopeIndexs[0], &scopeIndexs[arity - 1]);

    if (globalname == "knapsack") {
        string arguments;
        Long capacity = (myrandln() % (Long) costMax);
        if (capacity==0)
            capacity++;
        arguments.append(to_string(capacity));
        for (i = 0; i < arity; i++) {
            arguments.append(" ");
            if((rand() % 100)<50){
                Long weight = (myrandl() % capacity+1);
                arguments.append(to_string(weight));}
            else{
                Long weight = (-myrandl() % capacity-1);
                arguments.append(to_string(weight));}
        }
        istringstream file(arguments);
        wcsp.postKnapsackConstraint(scopeIndexs, arity, file);
    } else if (globalname == "salldiff" || globalname == "salldiffdp" || globalname == "walldiff") {
        wcsp.postWAllDiff(scopeIndexs, arity, "var", (globalname == "salldiff") ? "flow" : ((globalname == "walldiff") ? "network" : "DAG"), Top);
    } else if (globalname == "sgcc" || globalname == "sgccdp" || globalname == "wgcc") {
        // soft alldiff
        vector<BoundedObjValue> values;
        for (unsigned int i = 0; i < scopeVars[0]->getDomainInitSize(); i++) {
            values.push_back(BoundedObjValue(i, 1));
        }
        wcsp.postWGcc(scopeIndexs, arity, "var", (globalname == "sgcc") ? "flow" : ((globalname == "wgcc") ? "network" : "DAG"), Top, values);
    } else if (globalname == "sregular" || globalname == "sregulardp" || globalname == "wregular") {
        // random parity automaton (XOR)
        vector<WeightedObjInt> init(1, WeightedObjInt(0));
        vector<WeightedObjInt> last(1, WeightedObjInt(1));
        if (globalname == "wregular")
            last.push_back(WeightedObjInt(0, Top));
        vector<DFATransition> trans;
        for (unsigned int i = 0; i < scopeVars[0]->getDomainInitSize(); i++) {
            trans.push_back(DFATransition(0, i, (i % 2) ? 1 : 0));
            trans.push_back(DFATransition(1, i, (i % 2) ? 0 : 1));
        }
        wcsp.postWRegular(scopeIndexs, arity, "var", (globalname == "sregular") ? "flow" : ((globalname == "wregular") ? "network" : "DAG"), Top, 2, init, last, trans);
    } else {
        cerr << "Random generator: unknown global cost function name " << globalname << endl;
        exit(-1);
    }

    delete[] scopeIndexs;
    delete[] scopeVars;
}

void naryRandom::generateNaryCtr(vector<int>& indexs, long nogoods, Cost costMin, Cost costMax)
{
    int i;
    int arity = indexs.size();
    EnumeratedVariable** scopeVars = new EnumeratedVariable*[arity];
    int* scopeIndexs = new int[arity];

    Cost Top = wcsp.getUb();
    if (costMax < Top)
        Top = costMax;

    for (i = 0; i < arity; i++) {
        scopeIndexs[i] = indexs[i];
        scopeVars[i] = (EnumeratedVariable*)wcsp.getVar(indexs[i]);
    }

    Constraint* nctr = wcsp.getCtr(wcsp.postNaryConstraintBegin(scopeIndexs, arity, 0, nogoods));

    Tuple s(arity, 0);
    while (nogoods > 0) {
        for (i = 0; i < arity; i++)
            s[i] = myrand() % scopeVars[i]->getDomainInitSize();
        Cost c = ToulBar2::costMultiplier * randomCost(MIN_COST, costMax);
        nctr->setTuple(s, c);
        nogoods--;
    }
    nctr->propagate();

    delete[] scopeIndexs;
    delete[] scopeVars;
}

void naryRandom::generateTernCtr(int i, int j, int k, long nogoods, Cost costMin, Cost costMax)
{
    int a, b, c, dice;
    EnumeratedVariable* x = (EnumeratedVariable*)wcsp.getVar(i);
    EnumeratedVariable* y = (EnumeratedVariable*)wcsp.getVar(j);
    EnumeratedVariable* z = (EnumeratedVariable*)wcsp.getVar(k);
    int mx = x->getDomainInitSize();
    int my = y->getDomainInitSize();
    int mz = z->getDomainInitSize();
    int total_nogoods = mx * my * mz;

    vector<Cost> costs;
    for (a = 0; a < mx; a++)
        for (b = 0; b < my; b++)
            for (c = 0; c < mz; c++)
                costs.push_back(MIN_COST);

    while (nogoods > 0) {
        dice = myrand() % total_nogoods;
        for (a = 0; a < mx; a++)
            for (b = 0; b < my; b++)
                for (c = 0; c < mz; c++) {
                    if (costs[my * mz * a + b * mz + c] == MIN_COST) {
                        if (dice == 0) {
                            costs[my * mz * a + b * mz + c] = ToulBar2::costMultiplier * randomCost(costMin, costMax);
                            nogoods--;
                            total_nogoods--;
                            a = mx;
                            b = my;
                            c = mz;
                        }
                        dice--;
                    }
                }
    }
    wcsp.postTernaryConstraint(i, j, k, costs);
}

void naryRandom::generateSubModularBinCtr(int i, int j, Cost costMin, Cost costMax)
{
    int a, b;
    EnumeratedVariable* x = (EnumeratedVariable*)wcsp.getVar(i);
    EnumeratedVariable* y = (EnumeratedVariable*)wcsp.getVar(j);
    int mx = x->getDomainInitSize();
    int my = y->getDomainInitSize();

    vector<Cost> costs;
    for (a = 0; a < mx; a++)
        for (b = 0; b < my; b++)
            costs.push_back(MIN_COST);

    // row generation
    for (a = 0; a < mx; a++) {
        if (myrand() % 2) {
            Cost c = ToulBar2::costMultiplier * randomCost(costMin, costMax);
            for (b = 0; b < my; b++)
                costs[my * a + b] += c;
        }
    }
    // col generation
    for (b = 0; b < my; b++) {
        if (myrand() % 2) {
            Cost c = ToulBar2::costMultiplier * randomCost(costMin, costMax);
            for (a = 0; a < mx; a++)
                costs[my * a + b] += c;
        }
    }

    // rectangle generation
    int nrect = myrand() % mx;
    while (nrect) {
        Cost c = ToulBar2::costMultiplier * randomCost(costMin, costMax);
        int lx = myrand() % (mx - 1);
        int ly = myrand() % (my - 1);
        for (a = 0; a < lx; a++)
            for (b = 0; b < ly; b++) {
                costs[my * (mx - a - 1) + b] += c;
            }
        nrect--;
    }

    wcsp.postBinaryConstraint(i, j, costs);
}

void naryRandom::generateBinCtr(int i, int j, long nogoods, Cost costMin, Cost costMax)
{
    int a, b, dice;
    EnumeratedVariable* x = (EnumeratedVariable*)wcsp.getVar(i);
    EnumeratedVariable* y = (EnumeratedVariable*)wcsp.getVar(j);
    int mx = x->getDomainInitSize();
    int my = y->getDomainInitSize();
    int total_nogoods = mx * my;

    vector<Cost> costs;
    for (a = 0; a < mx; a++)
        for (b = 0; b < my; b++)
            costs.push_back(MIN_COST);

    while (nogoods > 0) {
        dice = myrand() % total_nogoods;
        for (a = 0; a < mx; a++)
            for (b = 0; b < my; b++) {
                if (costs[my * a + b] == MIN_COST) {
                    if (dice == 0) {
                        costs[my * a + b] = ToulBar2::costMultiplier * randomCost(costMin, costMax);
                        nogoods--;
                        total_nogoods--;
                        a = mx;
                        b = my;
                    }
                    dice--;
                }
            }
    }
    wcsp.postBinaryConstraint(i, j, costs);
}

long long naryRandom::toIndex(vector<int>& index)
{
    long long result = 1;
    for (int i = 0; i < (int)index.size(); i++)
        result += (long long)powl((double)n, i) * index[i];
    return result;
}

void naryRandom::ini(vector<int>& index, int arity, int n)
{
    index.clear();
    int dec = (arity>10)?(myrand()%(n-arity)):0;
    for (int i = 0; i < arity; i++)
        index.push_back(i + dec);
}

bool naryRandom::inc(vector<int>& index)
{
    int res = inc(index, index.size() - 1);
    if (res < 0)
        return false;
    else
        return true;
}

int naryRandom::inc(vector<int>& index, int i)
{
    if (i < 0)
        return i;
    assert(i < (int)index.size());

    index[i]++;
    if (index[i] == n - ((int)index.size() - i - 1)) {
        int val = inc(index, i - 1);
        if (val < 0)
            return -1;
        index[i] = val + 1;
        if (index[i] == n)
            return -1;
    }
    return index[i];
}

void naryRandom::Input(int in_n, int in_m, vector<int>& p, bool forceSubModular, string globalname)
{
    n = in_n;
    m = in_m;

    assert(p.size() >= 2);

    int i, arity;
    vector<int> indexs;
    vector<long> numCtrs;

    int maxa = p.size();

    for (arity = 0; arity <= maxa; arity++) {
        if (arity < 2)
            numCtrs.push_back(0);
        else
            numCtrs.push_back(p[arity - 1]);
    }

    if (forceSubModular) {
        numCtrs[maxa] = (numCtrs[maxa - 1] * numCtrs[maxa]) / 100;
        maxa--;
    }

    for (i = 0; i < n; i++) {
        string varname = to_string(i);
        wcsp.makeEnumeratedVariable(varname, 0, m - 1);
    }

    for (arity = maxa; arity > 1; arity--) {
        long nogoods = (long)(((double)p[0] / 100.) * pow((double)m, arity) + 0.5);
        //long totalarraysize = (long) pow( (double)n, arity);
        long long tCtrs = static_cast<long long>(n)*static_cast<long long>(n-1);
        set<long long> scopes;
        for (i = 2; i < arity; i++) {
            tCtrs *= (n - i);
            tCtrs /= i;
        }
        if (numCtrs[arity] > tCtrs) {
            cout << numCtrs[arity] << "  " << arity << "ary constraints and the maximum is " << tCtrs << endl;
            numCtrs[arity] = tCtrs;
        }

        while (numCtrs[arity]) {
            bool oneadded = false;
            int dice = max(0,min(MAX_ARITY*MAX_ARITY*MAX_ARITY, (int)((long long)myrand() % tCtrs)));
            ini(indexs, arity, n);
            do {
                if (scopes.end() == scopes.find(toIndex(indexs))) {
                    if (dice == 0) {
                        scopes.insert(toIndex(indexs));
                        if (arity > 1) {
                            switch (arity) {
                            case 2:
                                if (!forceSubModular || numCtrs[arity] > numCtrs[maxa + 1])
                                    generateBinCtr(indexs[0], indexs[1], nogoods);
                                else
                                    generateSubModularBinCtr(indexs[0], indexs[1], SMALL_COST, LARGE_COST);
                                break;
                            case 3:
                                generateTernCtr(indexs[0], indexs[1], indexs[2], nogoods);
                                break;
                            default:
                                if (globalname == "" || globalname == "nary")
                                    generateNaryCtr(indexs, nogoods);
                                else
                                    generateGlobalCtr(indexs, globalname);
                                break;
                            }
                        }
                        tCtrs--;
                        numCtrs[arity]--;
                        oneadded = true;
                    }
                    dice--;
                }
            } while (inc(indexs) && !oneadded);
        }
    }

    for (i = 0; i < n; i++) {
        EnumeratedVariable* x = (EnumeratedVariable*)wcsp.getVar(i);
        for (unsigned int a = 0; a < x->getDomainInitSize(); a++) {
            x->project(x->toValue(a), ToulBar2::costMultiplier * randomCost(MIN_COST, LARGE_COST), true);
        }
        x->findSupport();
    }

    if (forceSubModular) {
        for (i = 0; i < n; i++) {
            EnumeratedVariable* x = (EnumeratedVariable*)wcsp.getVar(i);
            x->permuteDomain(10);
        }
    }
}

/* Local Variables: */
/* c-basic-offset: 4 */
/* tab-width: 4 */
/* indent-tabs-mode: nil */
/* c-default-style: "k&r" */
/* End: */
