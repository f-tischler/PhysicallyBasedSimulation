/******************************************************************
*
* Scene.h
*
* Description: Class definition for scene description  
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __SCENE_H__
#define __SCENE_H__

#include <vector>
using namespace std;

#include "Spring.h"
#include "Point.h"

class Scene
{
public:
	/* Numerical solver */
	enum Method { EULER, SYMPLECTIC, LEAPFROG, MIDPOINT };

	Method method;

	/* Test scene */
	enum Testcase { SPRING, HANGING, FALLING };

	Testcase testcase;

private:
	/* Global simulation parameters */
	double step;
	double mass; /* Identical mass for all points */
	double stiffness; /* Identical spring stiffness for all springs */
	double damping; /* Identical damping for all points */
	bool interaction; /* Toggle for (hard-coded) external force */

protected:
	vector<Point> points;
	vector<Spring> springs;

public:
	Scene(void);
	Scene(int argc, char* argv[]);
	~Scene(void);

	void Init(void);
	void PrintSettings(void);
	void Render(); /* Draw scene */
	void Update(); /* Execute time step */

	double GetStep() const; /* Return time step */
	void ToggleUserForce(); /* Toggle external force On/Off */
};

#endif
