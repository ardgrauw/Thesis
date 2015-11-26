// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <C:\Ilog\include\ilcplex\ilocplex.h> 

//idea for multiple dimensions
//typedef IloArray<IloNumArray> Xjk;
//typedef IloArray<Xjk> Xijk;
//typedef IloArray<Xijk> Xijkl; //xijkl[l][i][j][k]



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
	double V; // volume capacity for a spoon
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
	for (IloInt i = 0; i < I; ++i){
		for (IloInt j = 0; j < J; ++j){
			for (IloInt k = 0; k < 2; ++k){
				IloNumVar w[i][j][k](env, 0, 50, ILOFLOAT);
				IloNumVar y[i][j][k](env, 0, 1, ILOBOOL);
				expr += w[i][j][k] * y[i][j][k] * c[i];
			}
		}
	}

	IloObjective obj(env, expr, IloObjective::Minimize);
	model.add(obj);
	expr.end();

	//Add constraints
	for (IloInt i = 0; i < I; ++i){
		for (IloInt j = 0; j < J; ++j){
			for (IloInt k = 0; k < 2; ++k){
				IloRange bigM[i][j][k](env, 0.3 * y[i][j][k], w[i][j][k], 50 * y[i][j][k]);
				IloRange stock[i][j][k](env, 0.0, w[i][j][k], A[i][j][k]);
				model.add(bigM[i][j][k]); model.add(stock[i][j][k]);
			}
		}
	}
	
	for (IloInt k = 0; k < 2; ++k){
		for (IloInt j = 0; j < J; ++j){
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
			IloRange Tijd[j][k](env, 0.0, tijd, t[j][k]);
			IloRange Gewicht[j][k](env, 0.0, gewicht, W);
			IloRange Volume[j][k](env, 0.0, volume, V);
			IloRange Types[j][k](env, 0.0, types, Y);
			model.add(Tijd[j][k]); model.add(Gewicht[j][k]); model.add(Volume[j][k]); model.add(Types[j][k]);
			tijd.end(); gewicht.end(); volume.end(); types.end();
		}
	}

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
		IloRange Gewicht[j](env, W[j], gewicht, totalWeight[j]);
		IloRange Residuelen[j](env, 0.0, residuelen, Re[j]);
		IloRange Zwavel[j](env, 0.0, zwavel, Su[j]);
		model.add(Gewicht[j]); model.add(Residuelen[j]); model.add(Zwavel[j]);
		gewicht.end(); residuelen.end(); zwavel.end();
	}

	//A initialiseren (nog steeds constraint)
	for (IloInt i = 0; i < I; ++i){
		for (IloInt k = 0; k < 2; ++k){
			A[i][0][k] = AStart[i][k];
			for (IloInt j = 0; j < J; ++j){
				A[i][j][k] = A[i][j - 1][k] - w[i][j - 1][k];
				IloRange Stock[i][j][k](env, 0.0, w[i][j][k], A[i][j][k]);
				model.add(Stock[i][j][k]);
			}
		}
	}


	//solve the model
	IloCplex(model);
	solve();

	env.end();
	
	return 0;
}

