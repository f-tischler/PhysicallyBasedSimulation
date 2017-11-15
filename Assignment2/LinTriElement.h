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
    int nodeID[3];       /* Global IDs of nodes */

public:
    LinTriElement(int node0, int node1, int node2)
    {
        nodeID[0] = node0;
        nodeID[1] = node1;
        nodeID[2] = node2;
    }
    int GetGlobalID(int elID) const { return nodeID[elID]; }

    void AssembleElement(FEModel *model) const;
    void ComputeBasisDeriv(int nodeId, Vector2 &basisDeriv, const FEModel *model) const;
};

#endif
