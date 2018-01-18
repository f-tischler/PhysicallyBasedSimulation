/******************************************************************
*
* FluidSim.cpp
*
* Description: This is an implementation of a 2D solver for the
* Euler Flow equations (i.e. reduced Navier-Stokes equations) for 
* an incompressible, Newtonian fluid. 
* Densities (smoke) advected in the flow field are rendered using 
* legacy OpenGL. Density is added either at a fixed location or
* via the mouse (left mouse button). The fluid density is assumed 
* constant as 1, approximately representing air.
* It is possible to toggle the boundary conditions (Dirichlet,
* Neumann - closed, open domain), the constant density inflow,
* and the injection of turbulence (vorticity).
* The problem domain is regularly subdivided into square fluid
* cells. No staggering is employed, i.e. central differences span 
* 2*dx. Pressures and velocity components are defined at the
* same locations.
* An integer as command line parameter gives the number of cells
* (including the boundary layer) per axis. The default is 200.
*
* Physically-Based Simulation Proseminar WS 2016
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

/* Standard includes */
#include <iostream>
#include <GL/glut.h> 

/* Local includes */
#include "Fluid2D.h"

using namespace std;

/*----------------------------------------------------------------*/
Fluid_2D* fluid;

bool pauseFlag = false;        /* Toggle simulation */
bool addSource = true;         /* Toggle density injection at fixed source */
bool addMouseSource = false;   /* Switch for adding density at mouse */

int width  = 600;
int height = 600;
double camera[] = {0.0, 1.0, 0.0, 1.0};

float xs = 0.0;    /* Coordinates of mouse for adding density */
float ys = 0.0;

/******************************************************************
*
*******************************************************************/

void reshapeCallback(int w, int h)
{
    if (h == 0) h = 1;
    
    width = w;
    height = h;

    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(camera[0], camera[1], camera[2], camera[3], -10.0, 10.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


/******************************************************************
* draw
*
* Display material density (smoke), advected in flow field;
* Density drawn in gray scale, normalized to [min,max] interval;
*******************************************************************/

static void draw(const int xRes, const int yRes, const double* density)
{
    glPushMatrix();
    glScalef(1.0 / (double)(xRes - 1), 1.0 / (double)(yRes - 1), 1.0);

    double max = 0.0;
    double min = fabs(density[1 + xRes]);

    for (int i = 0; i < xRes * yRes; i++)
    {
        max = fabs(density[i]) > max ? fabs(density[i]) : max;
        min = fabs(density[i]) < min ? fabs(density[i]) : min;
    }
    max = 1.0 / (max-min);

    for (int y = 0; y < yRes-1; y++)
        for (int x = 0; x < xRes-1; x++)
        {
            int index = x + y * xRes;

            /* Triangle fan for square cell with 5 nodes (center & corners) */
            glBegin(GL_TRIANGLE_FAN);
            {
                double SW = (fabs(density[index]) - min) * max;
                double SE = (fabs(density[index + 1]) - min) * max;
                double NW = (fabs(density[index + xRes]) - min) * max;
                double NE = (fabs(density[index + xRes + 1]) - min) * max;
                double average = (SW + SE + NW + NE) * 0.25;
                
                glColor4f(average, average, average, 1.0);
                glVertex3f(x + 0.5, y + 0.5, 0.0);

                glColor4f(SW, SW, SW, 1.0);
                glVertex3f(x, y, 0.0);

                glColor4f(NW, NW, NW, 1.0);
                glVertex3f(x, y+1, 0.0);

                glColor4f(NE, NE, NE, 1.0);
                glVertex3f(x+1, y+1, 0.0);

                glColor4f(SE, SE, SE, 1.0);
                glVertex3f(x+1, y, 0.0);

                glColor4f(SW, SW, SW, 1.0);
                glVertex3f(x, y, 0.0);
            }
            glEnd();
        }
    glPopMatrix();
}


/******************************************************************
*
*******************************************************************/

void displayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT);

    draw(fluid->get_xRes(), fluid->get_yRes(), fluid->get_density()); 

    glutSwapBuffers();
}


/******************************************************************
*
*******************************************************************/

void keybCallback(unsigned char key, int x, int y)
{
    switch(key) 
    {
        case 'q':
            exit(0);
            break;

        case 'd':  /* Toggle injection at fixed source */
            addSource = !addSource;
            break;

        case 'b':  /* Toggle boundary condition - Dirichlet, Neumann */
            fluid->toggleBoundaryCond();
            break;

        case 'v':  /* Toggle injection of turbulence */
            fluid->toggleVorticity();
            break;

        case 'c':  /* Clear density field (nothing else) */
            fluid->clearDensity();
            break;

        case ' ': 
            pauseFlag = !pauseFlag;
            break;
    }

    glutPostRedisplay();
}


/******************************************************************
*
*******************************************************************/

void buttonCallback(int button, int state, int x, int y)
{
    /* Add density when mouse button down */
    if((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN))
        addMouseSource = true;
    else
        addMouseSource = false; 

    glutPostRedisplay();
}


/******************************************************************
*
*******************************************************************/

void mouseCallback(int x, int y)
{
    /* Store mouse position for injecting density */
    xs = float(x) / width;
    ys = float(y) / height;
 
    if (xs < 0.0) xs = 0.01;
    if (xs > 1.0) xs = 0.99;

    if (ys < 0.0) ys = 0.01;
    if (ys > 1.0) ys = 0.99;

    glutPostRedisplay();
}


/******************************************************************
*
*******************************************************************/

void idleFunc()
{ 
    if (!pauseFlag)
    {
        if (addSource) /* Add fixed density source */
            fluid->addDensity(0.45, 0.55, 0.10, 0.15);

        if (addMouseSource) /* Add density at mouse position on button press */
            fluid->addDensity(xs-0.01, xs+0.01, 1.0-(ys+0.01), 1.0-(ys-0.01));

        /* Simulation time step */
        fluid->step(); 
    }

    glutPostRedisplay();
}


/******************************************************************
*
*******************************************************************/

int main(int argc, char **argv)
{
    /* Default simulation resolution */
    int sim_res = 200;

    if(argc == 2)
        sim_res = atoi(argv[1]);

    fluid = new Fluid_2D(sim_res, sim_res);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width, height);
    glutInitWindowPosition(100, 100);         
    glutCreateWindow("2D Fluid Simulation");

    glutDisplayFunc(displayCallback);
    glutReshapeFunc(reshapeCallback);
    glutKeyboardFunc(keybCallback);
	glutPassiveMotionFunc(mouseCallback);
	glutMotionFunc(mouseCallback);
    glutMouseFunc(buttonCallback);
    glutIdleFunc(idleFunc);  

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glutMainLoop();

    delete fluid;
    return 0;
}

