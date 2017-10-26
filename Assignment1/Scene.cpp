/******************************************************************
*
* Scene.cpp
*
* Description: Setup of 2D simulation scenes; three different scene
* configurations are hard-coded - a hanging mass, a hanging triangle,
* a falling triangle;  
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

/* Standard includes */
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <stdlib.h>
#include <iostream>

using namespace std;

/* Local includes */
#include "Scene.h"
#include "Point.h"
#include "Spring.h"
#include "Vec2.h"

/* External function for implementing the different numerical solvers */
extern void TimeStep(double dt, Scene::Method method,
                     vector<Point>& points, vector<Spring>& springs, bool userForce);
extern void reset_time(const double dt);

Scene::Scene(void)
{
	/* Default simulation parameters */
	testcase = SPRING;
	method = EULER;
	stiffness = 60.0;
	mass = 0.15;
	step = 0.003;
	damping = 0.08;
	interaction = false;


	initial_stiffness = stiffness;
	initial_mass = mass;
	initial_damping = damping;
	initial_step = step;
	Init();
	PrintSettings();
}

Scene::Scene(int argc, char* argv[])
{
	/* Default simulation parameters */
	testcase = SPRING;
	method = EULER;
	stiffness = 60.0;
	mass = 0.15;
	step = 0.003;
	damping = 0.08;
	interaction = false;

	/* Check for parameters in command line */
	int arg = 1;

	while (arg < argc)
	{
		/* Check testcase option */
		if (!strcmp(argv[arg], "-testcase"))
		{
			arg++;

			if (!strcmp(argv[arg], "spring1D"))
			{
				testcase = SPRING;
			}
			else if (!strcmp(argv[arg], "hanging"))
			{
				testcase = HANGING;
			}
			else if (!strcmp(argv[arg], "falling"))
			{
				testcase = FALLING;
			}
			else
			{
				cerr << "Unrecognized testcase: " << argv[arg] << endl;
				exit(1);
			}

			arg++;
		}

			/* Check for numerical integration method */
		else if (!strcmp(argv[arg], "-method"))
		{
			arg++;

			if (!strcmp(argv[arg], "euler"))
			{
				method = EULER;
			}
			else if (!strcmp(argv[arg], "symplectic"))
			{
				method = SYMPLECTIC;
			}
			else if (!strcmp(argv[arg], "leapfrog"))
			{
				method = LEAPFROG;
			}
			else if (!strcmp(argv[arg], "midpoint"))
			{
				method = MIDPOINT;
			}
			else
			{
				cerr << "Unrecognized method: " << argv[arg] << endl;
				exit(1);
			}

			arg++;
		}

			/* Check for step size */
		else if (!strcmp(argv[arg], "-step"))
		{
			step = (double)atof(argv[++arg]);
			arg++;
		}

			/* Check for stiffness */
		else if (!strcmp(argv[arg], "-stiff"))
		{
			stiffness = (double)atof(argv[++arg]);
			arg++;
		}

			/* Check for damping */
		else if (!strcmp(argv[arg], "-damp"))
		{
			damping = (double)atof(argv[++arg]);
			arg++;
		}

			/* Check for mass */
		else if (!strcmp(argv[arg], "-mass"))
		{
			mass = (double)atof(argv[++arg]);
			arg++;
		}

			/* Incorrect command line option; exit with message */
		else
		{
			cerr << endl << "Unrecognized option: " << argv[arg] << endl;
			cerr << "Usage: ./MassSpring -[option1] [setting1] -[option2] [setting2] ..." << endl;
			cerr << "Options:" << endl;
			cerr << "\t-testcase [spring, hanging, falling]" << endl;
			cerr << "\t-method [euler, symplectic, leapfrog, midpoint]" << endl;
			cerr << "\t-step [step size]" << endl;
			cerr << "\t-stiff [stiffness]" << endl;
			cerr << "\t-damp [damping]" << endl;
			cerr << "\t-mass [mass]" << endl << endl;
			exit(1);
			break;
		}
	}

	initial_stiffness = stiffness;
	initial_mass = mass;
	initial_damping = damping;
	initial_step = step;
	Init();
	PrintSettings();
}

Scene::~Scene(void)
{
}

void Scene::PrintSettings(void)
{
	cerr << endl << "Settings for testcase:" << endl;

	cerr << "\t-method ";
	switch (method)
	{
		case EULER:
			cerr << "euler" << endl;
			break;

		case SYMPLECTIC:
			cerr << "symplectic" << endl;
			break;

		case LEAPFROG:
			cerr << "leapfrog" << endl;
			break;

		case MIDPOINT:
			cerr << "midpoint" << endl;
			break;
	}

	cerr << "\t-mass " << mass << endl;
	cerr << "\t-step " << step << endl;
	cerr << "\t-stiff " << stiffness << endl;
	cerr << "\t-damp " << damping << endl << endl;
}

/******************************************************************
*
* Init
*
* Setup 2D simulation scenes; geometry for all three cases is 
* hard-coded
*
*******************************************************************/

void Scene::Init(void)
{
	Vec2 pt1(0.0, 1.0); /* Upper mass point for all example cases */
	Vec2 pt2; /* Temporary 2D vectors, initialized to (0,0) */
	Vec2 pt3;
	Vec2 ctr;
	Vec2 vec;

	if (testcase == SPRING)
	{
		pt2 = ctr; /* One additional mass point for single spring */
	}
	else
	{
		vec.x = cos(210.0 / 180.0 * M_PI);
		vec.y = sin(210.0 / 180.0 * M_PI);
		pt2 = ctr + vec; /* Second mass point of triangle geometry */

		vec.x = cos(330.0 / 180.0 * M_PI);
		vec.y = sin(330.0 / 180.0 * M_PI);
		pt3 = ctr + vec; /* Third mass point of triangle geometry */
	}

	/* Allocate the first two mass points, assuming same mass and damping */
	for (int i = 0; i < 2; i++)
		points.push_back(Point(mass, damping));

	/* Allocate the first spring */
	springs.push_back(Spring(stiffness));

	/* For cases with triangle geometry, allocate additional point and springs */
	if (testcase != SPRING)
	{
		points.push_back(Point(mass, damping));
		springs.push_back(Spring(stiffness));
		springs.push_back(Spring(stiffness));
	}

	/* Set actual position of first two mass points */
	points[0].setPos(pt1);
	points[1].setPos(pt2);

	/* Upper mass point fixed in first two example scenes */
	if (testcase != FALLING)
		points[0].setFixed(true);

	/* Assign points of first spring (rest length impicit) */
	springs[0].init(&points[0], &points[1]);

	/* For triangle examples, setup additional point and springs */
	if (testcase != SPRING)
	{
		points[2].setPos(pt3);

		/* Note: ordering of points for springs */
		springs[1].init(&points[1], &points[2]);
		springs[2].init(&points[2], &points[0]);

		/* Set external node force vector on one mass point */
		points[1].setUserForce(Vec2(0.5, 0.5));
	}
}

void Scene::Update(void)
{
	TimeStep(step, method, points, springs, interaction);
}

void Scene::Render(void)
{
	for (int i = 0; i < (int)springs.size(); i++)
		springs[i].render();

	for (int i = 0; i < (int)points.size(); i++)
		points[i].render();
}

double Scene::GetStep(void) const
{
	return step;
}

void Scene::ToggleUserForce(void)
{
	interaction = !interaction;
}

void Scene::resetInitial()
{
	mass = initial_mass;
	damping = initial_damping;
	stiffness = initial_stiffness;
	step = initial_step;
	points.clear();
	springs.clear();
	Init();
	PrintSettings();
	reset_time(0);
}

void Scene::increaseMass(const double value)
{

	this->mass+=value;
	points.clear();
	springs.clear();
	Init();
	PrintSettings();
	reset_time(0);
}

void Scene::increaseStiff(const double value)
{
	this->stiffness+=value;
	points.clear();
	springs.clear();
	Init();
	PrintSettings();
	reset_time(0);
}

void Scene::increaseDamp(const double value)
{
	this->damping+=value;
	points.clear();
	springs.clear();
	Init();
	PrintSettings();
	reset_time(0);
}

void Scene::increaseStep(const double value)
{
	this->step+=value;
	points.clear();
	springs.clear();
	Init();
	PrintSettings();
	reset_time(0);
}
