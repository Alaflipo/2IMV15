/*
  ======================================================================
   demo.c --- protoype to show off the simple solver
  ----------------------------------------------------------------------
   Author : Jos Stam (jstam@aw.sgi.com)
   Creation Date : Jan 9 2003

   Description:

	This code is a simple prototype that demonstrates how to use the
	code provided in my GDC2003 paper entitles "Real-Time Fluid Dynamics
	for Games". This code uses OpenGL and GLUT for graphics and interface

  =======================================================================
*/

#include "Particle.h"
#include "Object.h"
#include "FixedObject.h"
#include "RigidObject.h"
#include "Particle.h"
#include "FluidForce.h"
#include "SpringForce.h"
#include "GravityForce.h"

#include <stdlib.h>
#include <stdio.h>
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <vector>

/* macros */

#define IX(i,j) ((i)+(N+2)*(j))

/* external definitions (from solver.c) */

extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt );
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float * uVort, float * vVort,
                float visc, float dt, float eps, bool vc );
extern void addObjects ( std::vector<Object*> obj );
extern void rigidSimulationStep(std::vector<RigidObject*> rigidObjects, float dt);
extern void simulation_step(std::vector<Particle*> pVector, float timeStep, int integrationScheme);

/* global variables */

static int N;
static int dsim;
static float dt, diff, visc;
static float force, source;
static int dvel;

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;

static float * uVort, * vVort;
static float eps;
static bool vorticity_confinement;

static std::vector<Object *> objects;
static std::vector<RigidObject *> rigidObjects;
static std::vector<Particle*> particles;
static FluidForce * fluid_force;
static GravityForce * gravityForce;
static std::vector<SpringForce *> springForces;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;

static float timeStep = 0.01;
static int integrationScheme = 3;
static int runInstance = 1;


/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/


static void free_data ( void )
{
	// if ( u ) free ( u );
	// if ( v ) free ( v );
	// if ( u_prev ) free ( u_prev );
	// if ( v_prev ) free ( v_prev );
	// if ( uVort ) free ( uVort );
	// if ( vVort ) free ( vVort );
	// if ( dens ) free ( dens );
	// if ( dens_prev ) free ( dens_prev );

    particles.clear();
    objects.clear(); 

    springForces.clear();
    if (gravityForce) {
		delete gravityForce;
		gravityForce = NULL;
	}
    if (fluid_force) {
		delete fluid_force;
		fluid_force = NULL;
	}
}

static void clear_data ( void )
{
	int i, size=(N+2)*(N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = uVort[i] = vVort[i] = 0.0f;
	}

    int ii, particle_amount = particles.size();

	for(ii=0; ii<particle_amount; ii++){
		particles[ii]->reset();
	}

	for (RigidObject * rb: rigidObjects) {
		rb->reset();
	}
}

static void clearForces() {
	for (Particle * particle: particles) {
		particle->clearForce();
	}

	for (RigidObject * rb: rigidObjects) {
		rb->clearForce();
	}
}

static int allocate_data ( void )
{
	int size = (N+2)*(N+2);

	u			= (float *) malloc ( size*sizeof(float) );
	v			= (float *) malloc ( size*sizeof(float) );
	u_prev		= (float *) malloc ( size*sizeof(float) );
	v_prev		= (float *) malloc ( size*sizeof(float) );
	uVort		= (float *) malloc ( size*sizeof(float) );
	vVort		= (float *) malloc ( size*sizeof(float) );
	dens		= (float *) malloc ( size*sizeof(float) );	
	dens_prev	= (float *) malloc ( size*sizeof(float) );

	if ( !u || !v || !u_prev || !v_prev || !uVort || !vVort || !dens || !dens_prev ) {
		fprintf ( stderr, "cannot allocate data\n" );
		return ( 0 );
	}

	return ( 1 );
}


static void init_system(void)
{
    if (runInstance == 1) {
        Vec2f center(0.5, 0.5);
		const double dist = 0.2;
		std::vector <Vec2f> pointVector;
		pointVector.push_back(center + Vec2f(-0.1 + dist, -0.1));
		pointVector.push_back(center + Vec2f(0.1 + dist, -0.1));
		pointVector.push_back(center + Vec2f(0.0 + dist, 0.1));
		objects.push_back(new FixedObject(pointVector));

		center = Vec2f(0.2, 0.2);
		std::vector <Particle *> particles;
		particles.push_back(new Particle(center + Vec2f(dist, 0)));
		particles.push_back(new Particle(center + Vec2f(dist, dist)));
		particles.push_back(new Particle(center + Vec2f(0, dist)));
		particles.push_back(new Particle(center));
		RigidObject * rigidObject = new RigidObject(particles, N);
		objects.push_back(rigidObject);
		rigidObjects.push_back(rigidObject);

		addObjects(objects);
		fluid_force = new FluidForce(particles); 

    } else if (runInstance == 2) {
        const Vec2f center(0.5, 0.5);
        particles.push_back(new Particle(center));

        fluid_force = new FluidForce(particles); 
    } else if (runInstance == 3) {
        const float ks = 500.0;
		const float kd = 4;
		const int clothSize = 10;
		const double dist = 0.05;
		const Vec2f center(0.5, 0.5);

		for (int i = 0; i < clothSize; i++) {
			for (int j = 0; j < clothSize; j++) {
				Vec2f offset((i - (clothSize / 2)) * dist, (j - (clothSize / 2)) * dist);
				particles.push_back(new Particle(center + offset));
			}
		}

		for (int i = 0; i < clothSize; i++) {
			for (int j = 0; j < clothSize; j++) {
				if (i != clothSize - 1) {
					springForces.push_back(new SpringForce(particles[i * clothSize + j], particles[i * clothSize + j + 10], dist, ks, kd));
				}

				if (j != clothSize -1) {
					springForces.push_back(new SpringForce(particles[i * clothSize + j], particles[i * clothSize + j + 1], dist, ks, kd));
				}
			}
		}
        fluid_force = new FluidForce(particles); 
        gravityForce = new GravityForce(particles); 
    }
}

static void derivEval() {
	// Reset force_acc
	for (Particle * particle: particles) {
		particle->clearForce();
	}

    for (SpringForce * spring_force: springForces) {
		spring_force->calculateForce(false);
	}
	
	if (gravityForce) {
		gravityForce->calculateGravityForce(); 
	}

    fluid_force->calculateForce(N, dt, dens, u, v, u_prev, v_prev);

    // Run a step in the simulation 0 = Euler, 1 = Midpoint, 2 = Runge-Kutta, 3 = Implicit Euler
    // simulation_step( particles, timeStep, integrationScheme);

    if (runInstance == 3 || runInstance == 4) {
		particles[9]->reset();		// Fix top left point
		particles[99]->reset();		// Fix top right point
	}
}


/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	glutSwapBuffers ();
}

static void draw_velocity ( void )
{
	int i, j;
	float x, y, h;

	h = 1.0f/N;

	glColor3f ( 1.0f, 1.0f, 1.0f );
	glLineWidth ( 1.0f );

	glBegin ( GL_LINES );

		for ( i=1 ; i<=N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=1 ; j<=N ; j++ ) {
				y = (j-0.5f)*h;

				glVertex2f ( x, y );
				glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
			}
		}

	glEnd ();
}

static void draw_density ( void )
{
	int i, j;
	float x, y, h, d00, d01, d10, d11;

	h = 1.0f/N;

	glBegin ( GL_QUADS );

		for ( i=0 ; i<=N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=0 ; j<=N ; j++ ) {
				y = (j-0.5f)*h;

				d00 = dens[IX(i,j)];
				d01 = dens[IX(i,j+1)];
				d10 = dens[IX(i+1,j)];
				d11 = dens[IX(i+1,j+1)];

				glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
				glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
				glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
				glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
			}
		}

	glEnd ();
}

static void drawObjects() {
	for (Object * object: objects) {
		object->drawObject();
	}
}

static void draw_particles ( void )
{
	int size = particles.size();

	for(int ii=0; ii< size; ii++)
	{
		particles[ii]->draw();
	}
}

static void draw_forces ( void )
{
	if (springForces.size() > 0) {
		for (SpringForce * springForce: springForces) {
			springForce->draw();
		}
	}

    if (springForces.size() > 0) {
		for (SpringForce * springForce: springForces) {
			springForce->draw();
		}
	}

}

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

static void get_from_UI ( float * d, float * u, float * v )
{
	int i, j, size = (N+2)*(N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = d[i] = 0.0f;
	}

	if ( !mouse_down[0] && !mouse_down[2] ) return;

	i = (int)((       mx /(float)win_x)*N+1);
	j = (int)(((win_y-my)/(float)win_y)*N+1);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {
		u[IX(i,j)] = force * (mx-omx);
		v[IX(i,j)] = force * (omy-my);
	}

	if ( mouse_down[2] ) {
		d[IX(i,j)] = source;
	}

	omx = mx;
	omy = my;

	return;
}

static void remap_GUI()
{
	int ii, size = particles.size();
	for(ii=0; ii<size; ii++)
	{
		particles[ii]->reset();
	}
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
		case 'c':
		case 'C':
			clear_data ();
			break;

		case 'q':
		case 'Q':
			free_data ();
			exit ( 0 );
			break;

        case ' ':
            dsim = !dsim;
            std::cout << (dsim ? "Started animation\n" : "Paused animation\n");
        break;
		case 'v':
		case 'V':
			dvel = !dvel;
			break;
		case 'f':
		case 'F':
			vorticity_confinement = !vorticity_confinement;
			printf ( "Toggled vorticity confinement to %s\n", vorticity_confinement ? "True" : "False");
			break;
        case '1':
            free_data();
            runInstance = 1;
            dsim = false;
            std::cout << "Loaded 1st scene\n";
            init_system();
            break;
        case '2':
            free_data();
            runInstance = 2;
            dsim = false;
            std::cout << "Loaded second scene\n";
            init_system();
            break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	clearForces();

	get_from_UI ( dens_prev, u_prev, v_prev );
	vel_step ( N, u, v, u_prev, v_prev, uVort, vVort, visc, dt, eps, vorticity_confinement );
	dens_step ( N, dens, dens_prev, u, v, diff, dt );

	

    if ( dsim ) {
		// derivEval();
		fluid_force->calculateForce(N, dt, dens, u, v, u_prev, v_prev);
		rigidSimulationStep(rigidObjects, dt);
    } else {
        remap_GUI();
    }

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

		if ( dvel ) draw_velocity ();
		else		draw_density ();

	drawObjects();
    draw_particles(); 
    draw_forces(); 

	post_display ();
}


/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Alias | wavefront" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc != 1 && argc != 6 ) {
		fprintf ( stderr, "usage : %s N dt diff visc force source\n", argv[0] );
		fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
		fprintf ( stderr, "\t dt     : time step\n" );
		fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
		fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
		fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
		fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
		exit ( 1 );
	}

	if ( argc == 1 ) {
		N = 128;
		dt = 0.1f;
		diff = 0.0f;
		visc = 0.0f;
		force = 1.0f;
		source = 100.0f;
		eps = 10;
		vorticity_confinement = false;

		fprintf ( stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g vorticity confinement=%d\n",
			N, dt, diff, visc, force, source, vorticity_confinement );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		diff = atof(argv[3]);
		visc = atof(argv[4]);
		force = atof(argv[5]);
		source = atof(argv[6]);
	}

	printf ( "\n\nHow to use this demo:\n\n" );
	printf ( "\t Add densities with the right mouse button\n" );
	printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
	printf ( "\t Toggle density/velocity display with the 'v' key\n" );
	printf ( "\t Toggle vorticity confinement with the 'f' key\n" );
	printf ( "\t Clear the simulation by pressing the 'c' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dvel = 0;
    dsim = 0;

	if ( !allocate_data () ) exit ( 1 );
	clear_data ();

    init_system();

	win_x = 1024;
	win_y = 1024;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}