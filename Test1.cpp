// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

//Change directory naar cplex/include (en niet verder!)
#include "stdafx.h"
#include <ilcplex/ilocplex.h> 
#include <iostream>;

//idea for multiple dimensions
//typedef IloArray<IloNumArray> Xjk;
//typedef IloArray<Xjk> Xijk;
//typedef IloArray<Xijk> Xijkl; //xijkl[l][i][j][k]

//from IBM site:
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix>   NumVar3Matrix;
typedef IloArray<IloRangeArray> RangeMatrix;
typedef IloArray<RangeMatrix> Range3Matrix;
typedef IloArray<IloConstraintArray> ConstraintMatrix;
typedef IloArray<ConstraintMatrix> Constraint3Matrix;


int _tmain(int argc, _TCHAR* argv[]){
	//Variables
	const int J = 10; //number of convertor loads to do
	const int I = 10; // number of available types
	// constants also possible using #define const value
	double c[I]; // cost of scrap type i
	double V[I][2]; //loading speed
	double t[J][2]; // time to fill spoon for load j at place k
	double rho[I]; // density of type i
	double R[I]; // concentration of residuals in type i
	double S[I]; // concentration of sulfur in type i
	double Re[J]; // allowed quantity of residuals in load j
	double Su[J]; // allowed quantity of sulfur in load j
	double A[I][J][2]; // stock levels at the start of load J for type i at place k
	double AStart[I][2]; // initial stock levels
	double totalWeight[J]; // total weight for load j
	double W; // weight capacity for a spoon
	double Vol; // volume capacity for a spoon
	double Y; // number of types that the system can handle

	//test variables
	for (int i = 0; i < I; ++i){
		c[i] = i + 1;
		rho[i] = 0;
		R[i] = 1;
		S[i] = 1;
		for (int k = 0; k < 2; ++k){
			V[i][k] = 1;
			AStart[i][k] = 1000;
		}
	}
	for (int j = 0; j < J; ++j){
		Re[j] = 30;
		Su[j] = 30;
		for (int k = 0; k < 2; ++k){
			t[j][k] = 50;
		}
	}
	

	
	//make decision variables for subproblem and add to objective function
	IloEnv env;
	IloModel model(env);
	IloExpr expr(env);
	//gebruik IloInt in de for-loops?!?
	NumVar3Matrix w(env, I);
	NumVar3Matrix y(env, I);
	for (IloInt i = 0; i < I; ++i){
		w[i] = NumVarMatrix(env, J);
		y[i] = NumVarMatrix(env, J);
		for (IloInt j = 0; j < J; ++j){
			w[i][j] = IloNumVarArray(env, 2);
			y[i][j] = IloNumVarArray(env, 2);
			for (IloInt k = 0; k < 2; ++k){
				w[i][j][k] = IloNumVar (env, 0, 50, ILOFLOAT);
				y[i][j][k] = IloNumVar (env, 0, 1, ILOBOOL);
				expr += w[i][j][k] * y[i][j][k] * c[i];
			}
		}
	}

	IloObjective obj(env, expr, IloObjective::Minimize);
	model.add(obj);
	expr.end();

	//Add constraints
	Constraint3Matrix bigM(env, I);
	Range3Matrix stock(env, I);
	for (IloInt i = 0; i < I; ++i){
		bigM[i] = ConstraintMatrix(env, J);
		stock[i] = RangeMatrix(env, J);
		for (IloInt j = 0; j < J; ++j){
			bigM[i][j] = IloConstraintArray(env, 2);
			stock[i][j] = IloRangeArray(env, 2);
			for (IloInt k = 0; k < 2; ++k){
				bigM[i][j][k] = IloConstraint( w[i][j][k] <= 50 * y[i][j][k]);
				stock[i][j][k] = IloRange(env, 0.0, w[i][j][k], A[i][j][k]);
				model.add(bigM[i][j][k]); model.add(stock[i][j][k]);
			}
		}
	}
	
	RangeMatrix Tijd(env, J), Gewicht(env, J), Volume(env, J), Types(env, J);
	for (IloInt j = 0; j < J; ++j){
		Tijd[j] = IloRangeArray(env, 2); Gewicht[j] = IloRangeArray(env, 2);
		Volume[j] = IloRangeArray(env, 2); Types[j] = IloRangeArray(env, 2);
		for (IloInt k = 0; k < 2 ; ++k){
			IloExpr tijd(env);
			IloExpr gewicht(env);
			IloExpr volume(env);
			IloExpr types(env);
			for (IloInt i = 0; i < I; ++i){
				tijd += w[i][j][k] * V[i][k];
				gewicht += w[i][j][k];
				volume += w[i][j][k] * rho[i];
				types += y[i][j][k];
			}
			Tijd[j][k] = IloRange(env, 0.0, tijd, t[j][k]);
			Gewicht[j][k] = IloRange(env, 0.0, gewicht, W);
			Volume[j][k] = IloRange(env, 0.0, volume, Vol);
			Types[j][k] = IloRange(env, 0.0, types, Y);
			model.add(Tijd[j][k]); model.add(Gewicht[j][k]); model.add(Volume[j][k]); model.add(Types[j][k]);
			tijd.end(); gewicht.end(); volume.end(); types.end();
		}
	}

	IloRangeArray Residuelen(env, J);
	IloRangeArray Zwavel(env, J);
	IloConstraintArray Gewicht2(env, J);
	for (IloInt j = 0; j < J; ++j){
		IloExpr gewicht(env);
		IloExpr residuelen(env);
		IloExpr zwavel(env);
		for (IloInt i = 0; i < I; ++i){
			for (IloInt k = 0; k < 2; ++k){
				gewicht += w[i][j][k];
				residuelen += w[i][j][k] * R[i];
				zwavel += w[i][j][k] * S[i];
			}
		}
		Gewicht2[j] = IloConstraint(gewicht == totalWeight[j]);
		Residuelen[j] = IloRange(env, 0.0, residuelen, Re[j]);
		Zwavel[j] = IloRange(env, 0.0, zwavel, Su[j]);
		model.add(Gewicht[j]); model.add(Residuelen[j]); model.add(Zwavel[j]);
		gewicht.end(); residuelen.end(); zwavel.end();
	}

	//A initialiseren (nog steeds constraint)
	//Stock heeft als indeces i,k,j ipv i,j,k!
	Constraint3Matrix Stock(env, I);
	Constraint3Matrix Av(env, I);
	for (IloInt i = 0; i < I; ++i){
		Stock[i] = ConstraintMatrix(env, 2);
		Av[i] = ConstraintMatrix(env, 2);
		for (IloInt k = 0; k < 2; ++k){
			Stock[i][k] = IloConstraintArray(env, J);
			Av[i][k] = IloConstraintArray(env, J+1);
			Av[i][k][0] = IloConstraint(Av[i][k][0] == AStart[i][k]);
			Stock[i][k][0] = IloConstraint(w[i][0][k] <= Av[i][k][0]);
			for (IloInt j = 1; j < J+1; ++j){
				Av[i][j][k] = IloConstraint(Av[i][j][k] == Av[i][k][j - 1] - w[i][j - 1][k]);
				if (j != J + 1){
					Stock[i][k][j] = IloConstraint(w[i][j][k] <= Av[i][k][j]);
				}				
				model.add(Stock[i][j][k]);
			}
		}
	}


	//solve the model
	IloCplex cplex(model);
	cplex.solve();

	//output
	env.out() << "Solution status = " << cplex.getStatus();
	env.out() << "Solution value  = " << cplex.getObjValue();

	env.end();
	
	return 0;
}

