// Timing.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

//typedef om ook matrices to kunnen gebruiken (gemakkelijker dan enkel arrays te gebruiken)
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix>   NumVar3Matrix;
typedef IloArray<IloRangeArray> RangeMatrix;
typedef IloArray<RangeMatrix> Range3Matrix;
typedef IloArray<IloConstraintArray> ConstraintMatrix;
typedef IloArray<ConstraintMatrix> Constraint3Matrix;
//test


static void populatebynonzero(IloModel model, NumVarMatrix varOutput, NumVar3Matrix varHelp, Range3Matrix con);

const int J = 10; // how many loads should be considered
const int K = 2; // number of spoons in 1 load, should always stay 2
const int L = 6; // number of time variables to be considered
const int C = 20; // number of constraints
static double Tslack = 180; //slack time or margin time for all spoons
static int a[2] = { 2, 3 }; //differentiating between filling places for a certain constraint
static int b[2] = { 1, 2 }; //differentiating between filling places for a certain constraint

static double Tdc = 180; // driving time from drop-off to convertor (could be split up for both convertors)
static double Tcd = 180; // driving time from convertor to drop-off (could be split up for both convertors)
static double Tdf[2] = { 60, 300 }; // driving time from drop-off to the filling area (hall, dockside)
static double Tfd[2] = { 60, 420 }; // driving time from the filling area to drop-off (hall, dockside)
static double TlossD[2] = { 120, 180 }; // time lost between placing spoon at dropoff and picking up the next one
static double TlossF[2] = { 0, 60 }; // time lost between finishing one spoon and starting the next at the dockside
static double TloadMin[2] = { 780, 900 }; //minimal number of seconds that is required to load a spoon 

//test parameters
int current = 4;
static int T[15] = { 0, 1440, 2760, 5400, 6720, 8040, 9360, 10680, 12000, 13320, 15240, 16980, 18540, 20400, 21720 };
static int Tblow[15] = { 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60 };

int
main(int argc)
{


	IloEnv   env;
	try {
		IloModel model(env);

		NumVarMatrix varOutput(env, J + current);
		NumVar3Matrix varHelp(env, J + current);
		Range3Matrix cons(env, J + current);
		for (int j = 0; j <J + current; j++){
			varOutput[j] = IloNumVarArray(env, K);
			varHelp[j] = NumVarMatrix(env, K);
			cons[j] = RangeMatrix(env, K);
			for (int k = 0; k < K; k++){
				varOutput[j][k] = IloNumVar(env, 0.0, IloInfinity);
				varHelp[j][k] = IloNumVarArray(env, L);
				cons[j][k] = IloRangeArray(env, C);
				for (int l = 0; l < L; l++){
					varHelp[j][k][l] = IloNumVar(env, 0.0, IloInfinity);
				}
				if (j > current){
					cons[j][k][0] = IloRange(env, 0.0, 0.0);//will be used to express equality of varOutput, constraint (0)
					cons[j][k][1] = IloRange(env, 0.0, IloInfinity);// constraint (1)
					cons[j][k][2] = IloRange(env, -IloInfinity, T[j] - Tdc - Tblow[j] - Tslack);// constraint (2)
					cons[j][k][3] = IloRange(env, Tfd[k], Tfd[k]);// constraint (3)
					cons[j][k][4] = IloRange(env, 0.0, IloInfinity);// constraint (4)
					cons[j][k][5] = IloRange(env, Tdf[k], IloInfinity);// constraint (5)
					cons[j][k][6] = IloRange(env, T[j - a[k]] + Tcd, T[j - a[k]] + Tcd);// constraint (6)
					cons[j][k][7] = IloRange(env, TlossD[k], IloInfinity);// constraint (7)
					cons[j][k][8] = IloRange(env, TlossF[k], IloInfinity);// constraint (8)
				}
			}
		}

		populatebynonzero(model, varOutput, varHelp, cons);

		IloCplex cplex(model);

		// Optimize the problem and obtain solution.
		if (!cplex.solve()) {
			env.error() << "Failed to optimize LP" << endl;
			throw(-1);
		}

		IloNumArray vals(env);
		IloNumVar val(env);

		//vars to save output
		double TimeAvailable[J][K];
		double TimeInstances[J][K][L];
		double LK103[J][2];


		env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value  = " << cplex.getObjValue() << endl;
		for (int j = current; j < current + J; ++j)
		{
			cplex.getValues(vals, varOutput[j]);
			env.out() << "Seconds for load "<<j<<"       = " << vals << endl;
			/*for (int k = 0; k < K; k++){
				TimeAvailable[j][k] = cplex.getValue(varOutput[j][k]);
			}*/
		}
		for (int j = current; j < current + J; j++){
			for (int k = 0; k < K; k++){
				cplex.getValues(vals, varHelp[j][k]);
				env.out() << "Time instances for spoon "<<k<<" in load "<<j<<" = " << vals << endl;
				/*for (int l = 0; l < L; l++){
					TimeInstances[j][k][l] = cplex.getValue(varHelp[j][k][l]);
				}*/
			}
		}

		for (int j = current + 2; j < J + current; j++){
			LK103[j][0] = TimeInstances[j - 2][0][0];
			LK103[j][1] = TimeInstances[j][0][5];
			env.out() << "LK103, load " << j << " : " << LK103[j][1]-LK103[j][0] << endl;
		}
		/*cplex.getSlacks(vals, cons);
		env.out() << "Slacks        = " << vals << endl;
		cplex.getDuals(vals, cons);
		env.out() << "Duals         = " << vals << endl;
		cplex.getReducedCosts(vals, varOutput);
		env.out() << "Reduced Costs = " << vals << endl;*/

		cplex.exportModel("lpex1.lp");
	}
	catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "Unknown exception caught" << endl;
	}

	env.end();
	cin.get();
	return 0;
}  // END main


static void
populatebynonzero(IloModel model, NumVarMatrix varOutput, NumVar3Matrix varHelp, Range3Matrix con)
{
	IloEnv env = model.getEnv();

	IloObjective obj = IloMaximize(env); //maximization function


	for (int j = current; j < current + J; ++j)
	{
		for (int k = 0; k < K; ++k)
		{
			
			obj.setLinearCoef(varOutput[j][k], 1.0);//add all variables to objective function, factor 1 
			
			//constraint 0: express value of output objective variables
			model.add(varOutput[j][k] + varHelp[j][k][2] - varHelp[j][k][3] == 0);

			//constraint 1: Td2a>=+Td2b
			model.add(varHelp[j][k][5] - varHelp[j][k][4] >= 0);

			//constraint 2: Tj>=Td2a + Tdc + Tblow
			model.add(varHelp[j][k][5] <= T[j] - Tdc - Tblow[j] - Tslack);

			//constraint 3: Td2b = Tfa+Tfd
			model.add(Tfd[k] == varHelp[j][k][4] - varHelp[j][k][3]);

			//constraint 4: Td1a >= Td1b
			model.add(0 <= varHelp[j][k][1] - varHelp[j][k][0]);

			//constraint 5: Tfb >= Td1a+Tdf
			model.add(Tdf[k] <= varHelp[j][k][2] - varHelp[j][k][1]);

			//constraint 6: Td1b = T(j-a)+Tcd
			model.add(T[j - a[k]] + Tcd == varHelp[j][k][0]);

			//constraint 7: Td1a >= Td2b(j-b) + Tloss, 1
			model.add(TlossD[k] <= varHelp[j][k][1] - varHelp[j - b[k]][k][4]);

			//constraint 8: Tfb >= Tfa(j-1)+Tloss, 2
			model.add(TlossF[k] <= varHelp[j][k][2] - varHelp[j - 1][k][3]);

			//constraint 9: at least X s for every load
			model.add(varOutput[j][k] >= TloadMin[k]);

		}
		//constraint 10: both spoons are picked up at same time at dropoff: Td2a,1 == Td2a,2
		model.add(varHelp[j][1][5] == varHelp[j][0][5]);

	}

	model.add(obj);
	

}
