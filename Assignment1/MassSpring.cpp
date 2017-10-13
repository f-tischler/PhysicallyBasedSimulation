/******************************************************************
*
* MassSpring.cpp  
*
* Description: This file initializes the framework, sets up the
* rendering (using legacy OpenGL), and calls the scene update;
* it is possible to select a fixed amount of time steps per display
* frame (likely not running in real-time) or to attempt execution
* in real-time (requires setting of variable "steps_per_frame"
*
* Physically-Based Simulation Proseminar WS 2015
* 
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

/* Standard includes */
#include <GL/freeglut.h>
#include "Scene.h"

#include <chrono>
#include <ctime>

/*----------------------------------------------------------------*/

/* Simulation scene */
Scene* scene = NULL;

/* Time steps to be calculated per displayed output frame 
  (set to 0 to run simulation in real-time) */
static int steps_per_frame = 0;

/* Maximum time allowed between updates when running in real-time 
   mode (prevents performing too many calculations when running 
   slower than real time) */
static double max_update_time = 0.01;

/* Variables to keep track of timing information */
static unsigned long prevTime = 0;
static double remTime = 0;

/******************************************************************
*
* GetTime
*
* Returns curring time in milliseconds.
*
*******************************************************************/

unsigned long GetTime()
{
	/*struct timeval tv;
	gettimeofday(&tv, NULL);
	return (unsigned long)(tv.tv_sec * 1000 + tv.tv_usec);*/

	using chrono::duration_cast;
	using chrono::milliseconds;
	using chrono::system_clock;

	return static_cast<unsigned long>(duration_cast<milliseconds>(
			system_clock::now().time_since_epoch())
		.count());
}

/******************************************************************
*
* Display
*
* Update scene (thus executing simulation time step); either run a
* fixed amount of steps per visual frame (i.e. possibly running
* faster or slower than actual time) or attempt to execute as many
* steps as actual time window (i.e. execution in real-time);
*
* Note: the time to simulate a second in the physical simulation 
* may be smaller or larger than a second
*
*******************************************************************/

void Display(void)
{
	glClear(GL_COLOR_BUFFER_BIT);

	if (steps_per_frame > 0)
	{
		/* Fixed number of time steps per display frame */
		for (int i = 0; i < steps_per_frame; ++i)
			scene->Update();
	}
	else
	{
		/* Attempt to run simulation in real-time */
		unsigned long curTime = GetTime();
		double timePassed = (curTime - prevTime) / 1000.0;
		if (timePassed > max_update_time)
			timePassed = max_update_time;

		timePassed += remTime;

		int steps = (int)(timePassed / scene->GetStep());

		for (int i = 0; i < steps; i++)
			scene->Update();

		prevTime = curTime;
		remTime = fmod(timePassed, scene->GetStep());
	}

	/* Scene is rendered after simulation time step(s) */
	scene->Render();

	glutPostRedisplay();
	glutSwapBuffers();
}

/******************************************************************
*
* Reshape
*
* Adjust viewport when window is resized
*
*******************************************************************/

void Reshape(int width, int height)
{
	glViewport(0, 0, width, height);
}

/******************************************************************
*
* Idle
*
* Just asks for redisplay and thus for simulation time steps 
*
*******************************************************************/

void Idle(void)
{
	glutPostRedisplay();
}

/******************************************************************
*
* Init
*
* This function initializes the rendering; orthographic projection 
* is employed; a scene in 2D is assumed (x/y-coordinates) 
*
*******************************************************************/

void Init(void)
{
	/* Set background (clear) color to black */
	glClearColor(0.0, 0.0, 0.0, 0.0);

	/* Set appropriate mode for specifying projection matrix */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	/* Use orthographic projection */
	glOrtho(-3.0, 3.0, -3.0, 3.0, -3.0, 3.0);

	/* Back to modelview mode for object rendering */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Store start time */
	prevTime = GetTime();
}

/******************************************************************
*
* Keyboard
*
* Function to be called on key press in window; set by
* glutKeyboardFunc(); x and y specify mouse position on keypress;
* not used in this example 
*
*******************************************************************/

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 'q': case 'Q':
			exit(0);
			break;

		case 'f':
			/* Toggle (hard-coded) external force on a mass point */
			scene->ToggleUserForce();
			break;
	}

	glutPostRedisplay();
}

/******************************************************************
*
* main
*
* Main function to setup simulation scene, GLUT, and enter 
* rendering loop
*
*******************************************************************/

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(600, 600);
	glutCreateWindow("Mass-Spring Example");

	scene = new Scene(argc, argv);
	Init();
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Keyboard);
	glutIdleFunc(Idle);

	glutMainLoop();

	return EXIT_SUCCESS;
}
