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


static void populatebynonzero(IloModel model, NumVarMatrix varOutput, NumVar3Matrix varHelp, Range3Matrix con);

const int J = 10; // how many loads should be considered
const int K = 2; // number of spoons in 1 load, should always stay 2
const int L = 6; // number of time variables to be considered
static double Tslack = 180; //slack time or margin time for all spoons
static int a[2] = { 2, 3 }; //differentiating between filling places for a certain constraint
static int b[2] = { 1, 2 }; //differentiating between filling places for a certain constraint

static double Tdc = 180; // driving time from drop-off to convertor (could be split up for both convertors)
static double Tcd = 180; // driving time from convertor to drop-off (could be split up for both convertors)
static double Tdf[2] = { 60, 300 }; // driving time from drop-off to the filling area (hall, dockside)
static double Tfd[2] = { 60, 420 }; // driving time from the filling area to drop-off (hall, dockside)
static double TlossD[2] = { 120, 180 }; // time lost between placing spoon at dropoff and picking up the next one
static double TlossF[2] = { 0, 60 }; // time lost between finishing one spoon and starting the next at the dockside

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
				cons[j][k] = IloRangeArray(env, 10);
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
		env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value  = " << cplex.getObjValue() << endl;
		for (int j = current; j < current + J; ++j)
		{
			cplex.getValues(vals, varOutput[j]);
			env.out() << "Values        = " << vals << endl;
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

	//IloObjective obj = IloMaximize(env); //maximization function


	for (int j = current; j < current + J; ++j)
	{
		for (int k = 0; k < K; ++k)
		{
			/*con[j][k].add(IloRange(env, 0.0, 0.0));//will be used to express equality of varOutput, constraint (0)
			con[j][k].add(IloRange(env, 0.0, IloInfinity));// constraint (1)
			con[j][k].add(IloRange(env, -IloInfinity, T[j] - Tdc - Tblow[j] - Tslack));// constraint (2)
			con[j][k].add(IloRange(env, Tfd[k], Tfd[k]));// constraint (3)
			con[j][k].add(IloRange(env, 0.0, IloInfinity));// constraint (4)
			con[j][k].add(IloRange(env, Tdf[k], IloInfinity));// constraint (5)
			con[j][k].add(IloRange(env, T[j - a[k]] + Tcd, T[j - a[k]] + Tcd));// constraint (6)
			con[j][k].add(IloRange(env, TlossD[k], IloInfinity));// constraint (7)
			con[j][k].add(IloRange(env, TlossF[k], IloInfinity));// constraint (8)*/

			//varOutput[j].add(IloNumVar(env, 0.0, IloInfinity));//create decision variabes for available time
			//obj.setLinearCoef(varOutput[j][k], 1.0);//add all variables to objective function, factor 1 
			model.add(IloMaximize(env, IloSum(varOutput[j])));


			for (int l = 0; l < L; ++l)
			{
				//varHelp[j][k].add(IloNumVar(env, 0.0, IloInfinity));//define instances in time as variables as well
				//no need to include these in the objective function

			}

			//constraint 0: express value of output objective variables
			//con[j][k][0].setLinearCoef(varOutput[j][k], 1.0);
			//con[j][k][0].setLinearCoef(varHelp[j][k][3], -1.0);
			//con[j][k][0].setLinearCoef(varHelp[j][k][2], 1.0);
			model.add(varOutput[j][k] + varHelp[j][k][2] - varHelp[j][k][3] == 0);

			//constraint 1: Td2a>=+Td2b
			con[j][k][1].setLinearCoef(varHelp[j][k][4], -1.0);
			con[j][k][1].setLinearCoef(varHelp[j][k][5], 1.0);

			//constraint 2: Tj>=Td2a + Tdc + Tblow
			con[j][k][2].setLinearCoef(varHelp[j][k][5], 1.0);

			//constraint 3: Td2b = Tfa+Tfd
			con[j][k][3].setLinearCoef(varHelp[j][k][4], 1.0);
			con[j][k][3].setLinearCoef(varHelp[j][k][3], -1.0);

			//constraint 4: Td1a >= Td1b
			con[j][k][4].setLinearCoef(varHelp[j][k][1], 1.0);
			con[j][k][4].setLinearCoef(varHelp[j][k][0], -1.0);

			//constraint 5: Tfb >= Td1a+Tdf
			con[j][k][5].setLinearCoef(varHelp[j][k][2], 1.0);
			con[j][k][5].setLinearCoef(varHelp[j][k][1], -1.0);

			//constraint 6: Td1b = T(j-a)+Tcd
			con[j][k][6].setLinearCoef(varHelp[j][k][0], 1.0);

			//constraint 7: Td1a >= Td2b(j-b) + Tloss, 1
			con[j][k][7].setLinearCoef(varHelp[j][k][1], 1.0);
			con[j][k][7].setLinearCoef(varHelp[j - b[k]][k][4], -1.0);

			//constraint 8: Tfb >= Tfa(j-1)+Tloss, 2
			con[j][k][8].setLinearCoef(varHelp[j][k][2], 1.0);
			con[j][k][8].setLinearCoef(varHelp[j - 1][k][3], -1.0);

			model.add(con[j][k]);
		}
		//obj.setLinearCoefs(varOutput[j]);//add all variables to objective function, factor 1
	}

	//model.add(obj);
	

}
