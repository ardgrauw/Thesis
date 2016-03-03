// Scrap.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


// -------------------------------------------------------------- -*- C++ -*-
// File: examples/src/ilolpex1.cpp
// Version 8.1
// --------------------------------------------------------------------------
//  Copyright (C) 1999-2002 by ILOG.
//  All Rights Reserved.
//  Permission is expressly granted to use this example in the
//  course of developing applications that use ILOG products.
// --------------------------------------------------------------------------
//
// ilolpex1.cpp - Entering and optimizing a problem.  Demonstrates different
// methods for creating a problem.  The user has to choose the method
// on the command line:
//
//    ilolpex1  -r     generates the problem by adding rows
//    ilolpex1  -c     generates the problem by adding columns
//    ilolpex1  -n     generates the problem by adding a list of coefficients

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

//typedef om ook matrices to kunnen gebruiken (gemakkelijker dan enkel arrays te gebruiken)
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix>   NumVar3Matrix;
typedef IloArray<IloRangeArray> RangeMatrix;
typedef IloArray<RangeMatrix> Range3Matrix;
typedef IloArray<IloConstraintArray> ConstraintMatrix;
typedef IloArray<ConstraintMatrix> Constraint3Matrix;

//paar testvariabelen
const int S[10] = { 20, 30, 40, 25, 35, 30, 15, 60, 50, 35 }; //example of sulfur max
const int W[10] = { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 }; //needed weight
const int I = 10; // number of scrap types
const int J = 10; // number of convertor loads
const int K = 1; // number of stocks to implement
const int T = 10; //number of time variables
const int s[10] = { 3, 4, 2, 5, 2, 6, 1, 5, 4, 3 }; //example of sulfur concentration 
const int cost[10] = { 3, 2, 4, 1, 4, 0, 5, 4, 3, 3 }; // example of cost function




static void populatebynonzero(IloModel model, IloNumVarArray var, IloRangeArray con);
static void populatebynonzero(IloModel model,
		NumVar3Matrix varW, NumVar3Matrix varY, NumVar3Matrix varX, NumVar3Matrix varT, NumVar3Matrix varA,
		RangeMatrix con);


int
main(int argc)
{
	

	IloEnv   env;
	try {
		IloModel model(env);

		IloNumVarArray var(env);
		NumVar3Matrix varW(env);
		NumVar3Matrix varY(env);
		NumVar3Matrix varX(env);
		NumVar3Matrix varT(env);
		NumVar3Matrix varA(env);
		IloRangeArray con(env);
		RangeMatrix c(env);

		populatebynonzero(model, varW, varY, varX, varT, varA, c);

		IloCplex cplex(model);

		// Optimize the problem and obtain solution.
		if (!cplex.solve()) {
			env.error() << "Failed to optimize LP" << endl;
			throw(-1);
		}

		IloNumArray vals(env);
		env.out() << "Solution status = " << cplex.getStatus() << endl;
		env.out() << "Solution value  = " << cplex.getObjValue() << endl;
		for (int i = 0; i < I; i++)
		{
			for (int j = 0; j < J; j++)
			{
				cplex.getValues(vals, varW[i][j]);
				env.out() << "Values        = " << vals << endl;
			}
		}
		cplex.getSlacks(vals, con);
		env.out() << "Slacks        = " << vals << endl;
		cplex.getDuals(vals, con);
		env.out() << "Duals         = " << vals << endl;
		cplex.getReducedCosts(vals, var);
		env.out() << "Reduced Costs = " << vals << endl;

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

// populatebynonzero will define the whole problem,
// it enters the objective function, constraints and variables into the model

static void
populatebynonzero(IloModel model,
NumVar3Matrix varW, NumVar3Matrix varY, NumVar3Matrix varX, NumVar3Matrix varT, NumVar3Matrix varA,
RangeMatrix con)
{
	void populateR(IloModel model, NumVar3Matrix var),
		populateB(IloModel model, NumVar3Matrix var),
		populateT(IloModel model, NumVar3Matrix var);

	IloEnv env = model.getEnv();

	IloObjective obj = IloMinimize(env); //minimization function

	populateR(model, varW);
	populateR(model, varX);
	populateR(model, varA);
	populateB(model, varY);
	populateT(model, varT);

	for (int j = 0; j < J; j++)
	{
		con[j][0] = IloRange(env, -IloInfinity, S[j]);
		con[j][1] = IloRange(env, W[j], W[j]);

		for (int i = 0; i < I; i++)
		{
			for (int k = 0; k < K; k++)
			{
				obj.setLinearCoef(varW[i][j][k], cost[i]);
				con[j][0].setLinearCoef(varW[i][j][k], s[i]);
				con[j][1].setLinearCoef(varW[i][j][k], 1.0);
			}
		}

		model.add(con[j]);
	}
	
	model.add(obj);
	

}


//function to insert real decision variables (but greater then or equal to 0)
void
populateR(IloModel model, NumVar3Matrix var)
{
	IloEnv env = model.getEnv();
	for (int i = 0; i < I; i++)
	{
		for (int j = 0; j < J; j++)
		{
			for (int k = 0; k < K; k++)
			{
				var[i][j].add(IloNumVar(env, 0.0, IloInfinity));
			}
		}
	}
}

//function to insert binary decision variables 
void
populateB(IloModel model, NumVar3Matrix var)
{
	IloEnv env = model.getEnv();
	for (int i = 0; i < I; i++)
	{
		for (int j = 0; j < J; j++)
		{
			for (int k = 0; k < K; k++)
			{
				var[i][j][k] = IloIntVar(env, 0, 1);
			}
		}
	}
}

//function to insert time decision variables 
void
populateT(IloModel model, NumVar3Matrix var)
{
	IloEnv env = model.getEnv();
	for (int t = 0; t < T; t++)
	{
		for (int j = 0; j < J; j++)
		{
			for (int k = 0; k < K; k++)
			{
				var[t][j][k] = IloNumVar(env, 0.0, IloInfinity);
			}
		}
	}
}


// To populate by nonzero, we first create the rows, then create the
// columns, and then change the nonzeros of the matrix 1 at a time.
// This is the standard function, test purposes only

static void
populatebynonzero(IloModel model, IloNumVarArray x, IloRangeArray c)
{
	IloEnv env = model.getEnv();

	IloObjective obj = IloMinimize(env); //minimization function
	c.add(IloRange(env, -IloInfinity, S[1]));
	c.add(IloRange(env, 10.0, 10.0));

	for (int i = 0; i < I; ++i){
		x.add(IloNumVar(env, 0.0, IloInfinity));
	}

	for (int i = 0; i < I; ++i){
		obj.setLinearCoef(x[i], cost[i]);
		c[0].setLinearCoef(x[i], s[i]);
		c[1].setLinearCoef(x[i], 1.0);
	}

	model.add(obj);
	model.add(c);

	/* IloObjective obj = IloMaximize(env);
	c.add(IloRange(env, -IloInfinity, 20.0));
	c.add(IloRange(env, -IloInfinity, 30.0));

	x.add(IloNumVar(env, 0.0, 40.0));
	x.add(IloNumVar(env));
	x.add(IloNumVar(env));

	obj.setLinearCoef(x[0], 1.0);
	obj.setLinearCoef(x[1], 2.0);
	obj.setLinearCoef(x[2], 3.0);

	c[0].setLinearCoef(x[0], -1.0);
	c[0].setLinearCoef(x[1], 1.0);
	c[0].setLinearCoef(x[2], 1.0);
	c[1].setLinearCoef(x[0], 1.0);
	c[1].setLinearCoef(x[1], -3.0);
	c[1].setLinearCoef(x[2], 1.0);

	model.add(obj);
	model.add(c); */

}  // END populatebynonzero


