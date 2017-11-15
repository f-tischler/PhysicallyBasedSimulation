/******************************************************************
*                                                                  
* LinTriElement.cpp
*
* Description: Implementation of functions for handling linear 
* triangular elements
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#include <math.h>
#include "FEModel.h"

void LinTriElement::ComputeBasisDeriv(int nodeId, /*Vector2*/ Vector3& basisDeriv, const FEModel *model) const
{
    auto v1 = model->GetNodePosition(GetGlobalID(0));
    auto v2 = model->GetNodePosition(GetGlobalID(1));
    auto v3 = model->GetNodePosition(GetGlobalID(2));

    double x1 = v1.x;
    double x2 = v2.x;
    double x3 = v3.x;

    double y1 = v1.x;
    double y2 = v2.x;
    double y3 = v3.x;

    Matrix3x3T P = Matrix3x3T{};
    P(0,0) = 1; 
    P(1,0) = x1; 
    P(2,0) = y1;

    P(0,1) = 1; 
    P(1,1) = x2; 
    P(2,1) = y2;

    P(0,2) = 1; 
    P(1,2) = x3; 
    P(2,2) = y3;

    // Task 1
    double triangleArea = 0.5*P;

    Vector3 vc;
    vc.x = (x2*y3 - x3*y2)/(2*triangleArea);
    vc.y = (y2 - y3)/(2*triangleArea);
    vc.z = (x3 - x2)/(2*triangleArea);


    basisDeriv = vc;
}

void LinTriElement::AssembleElement(FEModel *model) const
{
    // Task 2
}

