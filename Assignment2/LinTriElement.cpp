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


Matrix3x3 LinTriElement::getP(const FEModel* model) const
{
    auto v1 = model->GetNodePosition(GetGlobalID(0));
    auto v2 = model->GetNodePosition(GetGlobalID(1));
    auto v3 = model->GetNodePosition(GetGlobalID(2));

    double x1 = v1[0];
    double x2 = v2[0];
    double x3 = v3[0];

    double y1 = v1[1];
    double y2 = v2[1];
    double y3 = v3[1];

    Matrix3x3T<double> P{};
    
    if(false)
    {
        P(0,0) = 1; 
        P(1,0) = x1; 
        P(2,0) = y1;
    
        P(0,1) = 1; 
        P(1,1) = x2; 
        P(2,1) = y2;
    
        P(0,2) = 1; 
        P(1,2) = x3; 
        P(2,2) = y3;
    }
    else
    {
        P(0, 0) = 1; 
        P(0, 1) = x1; 
        P(0, 2) = y1;
    
        P(1, 0) = 1; 
        P(1, 1) = x2; 
        P(1, 2) = y2;
    
        P(2, 0) = 1; 
        P(2, 1) = x3; 
        P(2, 2) = y3;
    }

    return P;
}

Matrix3x3 LinTriElement::getConstants(const FEModel* model) const
{
    Matrix3x3 P = getP(model);
    Matrix3x3 C = P.Inverse();
    return C;
}

double LinTriElement::getN(int j, const FEModel* model) const
{
    auto& position = model->GetNodePosition(GetGlobalID(j));
    auto x = position[0];
    auto y = position[1];

    Matrix3x3T<double> c = getConstants(model);
    return c(1, 0) + c(1,1)*x + c(1,2)*y;
}

double LinTriElement::getArea(FEModel* model) const
{
    return 0.5 * getP(model).Det();
}

void LinTriElement::ComputeBasisDeriv(int nodeId, Vector2& basisDeriv, const FEModel *model) const
{
    Matrix3x3T<double> c = getConstants(model);
    basisDeriv[0] = c(nodeId,1);
    basisDeriv[1] = c(nodeId,2);
}

void LinTriElement::AssembleElement(FEModel *model) const
{
    // Task 2
    double triangleArea = getArea(model);

    Matrix3x3T<double> stiffnessMatrix{};

    for(int i = 0; i < 3; i++)
    {    
        for(int j = 0; j < 3; j++)
        {
            double elemMatSum = 0.0;

            for(int ii = i; ii< 3; ii++)
            {    
                for(int jj = j; jj < 3; jj++)
                {
                    Vector2 basisDeriveI;
                    ComputeBasisDeriv(i, basisDeriveI, model);
            
                    Vector2 basisDeriveJ;
                    ComputeBasisDeriv(j, basisDeriveJ, model);
        
                    elemMatSum += basisDeriveI.x() * basisDeriveJ.x() + basisDeriveI.y() * basisDeriveJ.y();
                }
            }

            double elemValue = elemMatSum * getArea(model);
            model->AddToStiffnessMatrix(i, j, elemValue);
        }
    }
}

