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

void LinTriElement::ComputeBasisDeriv(int nodeId, Vector2 &basisDeriv) const
{
    // Task 1
    basisDeriv[0] = m_coefMat(1, nodeId);
    basisDeriv[1] = m_coefMat(2, nodeId);
}

void LinTriElement::AssembleElement(FEModel *model)
{
    setup(model);
    // Task 2

    Vector2 dervI, derivJ;
    for (int i = 0; i < 3; i++) 
    {
        ComputeBasisDeriv(i, dervI);
        for (int j = 0; j < 3; j++) 
        {
            ComputeBasisDeriv(j, derivJ);

            double elemValue = dervI[0] * derivJ[0] + dervI[1] * derivJ[1];
            elemValue *= m_area;

            model->AddToStiffnessMatrix(GetGlobalID(i), GetGlobalID(j), elemValue);
        }
    }
}

double LinTriElement::getNj(int j) const
{
    return m_coefMat(0, j) 
          + m_coefMat(1, j) * m_center.x() 
          + m_coefMat(2, j) * m_center.y();
}

void LinTriElement::setup(FEModel* model)
{
    const Vector2& v1 = model->GetNodePosition(GetGlobalID(0));
    const Vector2& v2 = model->GetNodePosition(GetGlobalID(1));
    const Vector2& v3 = model->GetNodePosition(GetGlobalID(2));

    Matrix3x3 areaMat;
    areaMat(0,0) = v1.x();
    areaMat(0,1) = v1.y();
    areaMat(0,2) = 1.;
    
    areaMat(1,0) = v2.x();
    areaMat(1,1) = v2.y();
    areaMat(1,2) = 1.;

    areaMat(2,0) = v3.x();
    areaMat(2,1) = v3.y();
    areaMat(2,2) = 1.;

    Matrix3x3 coefMat;
    coefMat(0,0) = 1.;
    coefMat(0,1) = v1.x();
    coefMat(0,2) = v1.y();
    
    coefMat(1,0) = 1.;
    coefMat(1,1) = v2.x();
    coefMat(1,2) = v2.y();

    coefMat(2,0) = 1.;
    coefMat(2,1) = v3.x();
    coefMat(2,2) = v3.y();

    m_area = areaMat.Det() * 0.5;
    m_center = (v1 + v2 + v3) / 3.;
    m_coefMat = coefMat.Inverse();
}
