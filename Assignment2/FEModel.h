/******************************************************************
*                                                                  
* FEModel.h
*
* Description: Class definition for Finite Element Model and
* Boundary Conditions
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __FE_MODEL_H__
#define __FE_MODEL_H__

#include "PCGT.h"
#include "Vec2.h"
#include "LinTriElement.h"
#include <vector>

/*----------------------------------------------------------------*/
class BoundaryCondition
{
public:
    BoundaryCondition(int _nodeID, double _value) 
    {
        nodeID = _nodeID;     /* Global node ID */
        value = _value;       /* Value on boundary */
    }
    int GetID() const { return nodeID; }
    double GetValue() const { return value; }

private:
    int nodeID;
    double value;
};

/*----------------------------------------------------------------*/
class FEModel
{
private:
    vector<Vector2> nodes;            /* Coordinates of vertices */
    vector<LinTriElement> elements;   /* Triangular elements */
    SparseSymmetricMatrix K_matrix;
    vector<double> rhs;               /* Right-hand side */

    vector<BoundaryCondition> boundaryConds;

    vector<double> solution;          /* Unknown nodal values */
    vector<double> error;             /* Error vector */
    vector<double> abserror;          /* Absolute error */

    int num_nodes;                    /* Number of nodes */
    int num_elems;                    /* Number of elements */

public:
    FEModel(void)
    {
        num_nodes = 0;
        num_elems = 0;
    }

    virtual const Vector2 &GetNodePosition(int nodeID) const 
    {
        return nodes[nodeID];
    }
    
    virtual void AddToStiffnessMatrix(int i, int j, double val) 
    {
    	/* The solver expects a lower triangular matrix */
    	if (j <= i)
        	K_matrix(i,j) += val;
    }

    void CreateUniformGridMesh(int nodesX, int nodesY);

    void AssembleStiffnessMatrix();
    void SetBoundaryConditions();
    void ComputeRHS();   
    
    void Solve();
    double ComputeError();

    void Render(int toggle_vis);   
};

#endif
