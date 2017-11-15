/******************************************************************
*
* PCGT.h
*
* Description: Code implements a diagonally preconditioned conjugate 
* gradient solver.
*
* Solves linear system A*x = b for unknown vector x. 
* Matrix A must be symmetric and positive-definite.
*
* From: Jonathan Richard Shewchuk, "An Introduction to the Conjugate 
* Gradient Method Without the Agonizing Pain"
* http://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf

* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __PCGT_T_H__
#define __PCGT_T_H__

#include <iostream>
#include <vector>

#include "Vec2.h"
#include "Vec3.h"
#include "SparseSymMat.h"

using namespace std;

template<class T>
class SparseLinSolverPCGT
{
public:

/* residual: desired accuracy of solution
   maxIterations: maximum number of iterations to perform 
                  (-1: infinite amount of iterations) */

    void SolveLinearSystem(SparseSymmetricMatrixT<T> &matA, 
                           vector<T> &x, const vector<T> &b, 
                           T residual, int maxIterations) 
    {
        int n = matA.GetNumRows();
        
        vector<T> precond(n);
        vector<T> r(n);
        vector<T> d(n);
        vector<T> q(n);
        vector<T> s(n);
        
        for(int i=0; i<n; i++)
            precond[i] = 1 / matA(i, i);

        matA.MultVector(x, r);
        for(int i=0; i<n; i++)
            r[i] = b[i] - r[i];

        for(int i=0; i<n; i++)
            d[i] = precond[i] * r[i];
       
        T deltaNew = dotProd(r, d);      
        T delta0 = 1.0; 
        
        int iter = 0;
        while(maxIterations == -1 || iter < maxIterations)
        {
            if(deltaNew <= residual*residual*delta0)
                break;

            matA.MultVector(d, q);
           
            T alpha = deltaNew / dotProd(d, q);

            for(int i=0; i<n; i++)
                x[i] += alpha*d[i];

            for(int i=0; i<n; i++)
                r[i] -= alpha*q[i];

            for(int i=0; i<n; i++)
                s[i] = precond[i] * r[i];

            T deltaOld = deltaNew;

            deltaNew = dotProd(r, s);

            T beta = deltaNew / deltaOld;

            for(int i=0; i<n; i++)
                d[i] = s[i] + beta*d[i];

            iter++;
            cout << "PCG, iter=" << iter << ", deltaNew=" 
                 << sqrt(deltaNew) << " vs "<< (residual) <<"\n";
        }   
    }

private:
    static T dotProd(const vector<T> &a, const vector<T> &b) 
    {
        T v = 0;
        
        for(int i=0; i<(int)a.size(); i++)
            v += a[i] * b[i];
        
        return v;
    }
};

#endif
