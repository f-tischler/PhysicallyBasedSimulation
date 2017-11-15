/******************************************************************
*
* FEM.cpp
*
* Description: This file initializes the Finite Element model, calls
* the solver, and then renders the result (using legacy OpenGL). It
* is possible to toggle between displaying the solution and the 
* absolute error. The final error norm is also printed. 
* 
* The problem domain is regularly subdivided with triangles. An integer
* as command line parameter gives the number of elements (2x) per axis.
* The standard is 20.
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

/* Standard includes */
#include <iostream>
#include "GL/glut.h"  

/* Local includes */
#include "FEModel.h"

/*----------------------------------------------------------------*/
bool toggle_vis = true;         /* Toggle between display of solution and error */ 

FEModel model;


/******************************************************************
*
*******************************************************************/

void keyboard(unsigned char key, int x, int y) 
{
    switch (key) 
    {
        case 'q': 
        case 27: 
            exit(0);
            break;

        case 'v':
            toggle_vis = !toggle_vis;
            std::cout << "toggle visualization: " << (toggle_vis ? "true" : "false") << std::endl;
            break;
    }

    glutPostRedisplay();
}


/******************************************************************
*
*******************************************************************/

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    /* Transform [0,1]x[0,1] region to standard [-1,1]x[-1,1] */
    glTranslatef(-1.0, -1.0, 0.0);
    glScalef(2.0, 2.0, 1.0);   

    model.Render(toggle_vis);     
   
    glPopMatrix();
    glutSwapBuffers();
}


/******************************************************************
*
*******************************************************************/

void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
}


/******************************************************************
*
*******************************************************************/

int main(int argc, char *argv[])
{
    /* Mesh resoluion: gridxgridx2 triangles */
    int grid = 20;    

    if(argc == 2)
        grid = atoi(argv[1]);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    model.CreateUniformGridMesh(grid, grid);   

    model.AssembleStiffnessMatrix();           
    model.ComputeRHS();
    model.SetBoundaryConditions();
    
    model.Solve();

    double err_nrm = model.ComputeError();
    cout << "Error norm is " << err_nrm << endl;

    glutCreateWindow("FEM");
    glutReshapeWindow(600, 600);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMainLoop();

    return 0;
}


