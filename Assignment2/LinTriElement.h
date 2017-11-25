/******************************************************************
*                                                                  
* LinTriElement.h
*
* Description: Class definition for linear triangular element
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __LIN_TRI_ELEMENT_H__
#define __LIN_TRI_ELEMENT_H__

#include "Vec2.h"
#include "Vec3.h"
#include "Mat3x3.h"


class FEModel;  /* Forward declaraion of class FEModel */

class LinTriElement
{
private:
    int m_nodeID[3];       /* Global IDs of nodes */
    Vector2 m_center;
    Matrix3x3 m_coefMat;
    double m_area;

public:
    LinTriElement(int node0, int node1, int node2)
    {
        m_nodeID[0] = node0;
        m_nodeID[1] = node1;
        m_nodeID[2] = node2;
    }
    int GetGlobalID(int elID) const { return m_nodeID[elID]; }

    void AssembleElement(FEModel *model);
    void ComputeBasisDeriv(int nodeId, Vector2 &basisDeriv) const;

    const Matrix3x3& getCoefMat() { return m_coefMat; }
    const Vector2& getCenter() { return m_center; }
    double getArea() { return m_area; }
    double getNj(int j) const;

private:
    void setup(FEModel* model);
};

#endif
