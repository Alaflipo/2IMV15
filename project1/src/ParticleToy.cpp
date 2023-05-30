// ParticleToy.cpp : Defines the entry point for the console application.
//
#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "GravityForce.h"
#include "CircularWireConstraint.h"
#include "imageio.h"
#include "ConstraintSolver.h"
#include "Constraint.h" 
#include "AngularSpringForce.h"
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#if defined(__CYGWIN__) || defined(WIN32)
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <LineConstraint.h>

#define PI 3.1415926535897932384626433832795

/* macros */

/* external definitions (from solver) */
extern void simulation_step(std::vector<Particle*> pVector, float dt, int scheme);

/* global variables */

static int N;
static float dt, d;
static int scheme; 
static int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static std::vector<Particle*> pVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

static bool activeMouseParticle;

static GravityForce * gravity_force;

static std::vector<SpringForce *> springForces;
static std::vector<AngularSpringForce *> angularSpringForces;
static std::vector<CircularWireConstraint *> circularWireConstraints;
static std::vector<RodConstraint *> rodConstraints;

static std::vector<Constraint *> constraints;
static ConstraintSolver * constraintSolver;

static std::vector<Particle*> endpoints; 

static bool dude = false; 
static bool particle_draw = true; 

static bool gravity = true; 
static int runInstance = 1;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	pVector.clear();
    dude = false;
    particle_draw = true; 

	springForces.clear();
    angularSpringForces.clear(); 
	circularWireConstraints.clear();
	rodConstraints.clear();
    endpoints.clear();
    if (gravity_force) {
		delete gravity_force;
		gravity_force = NULL;
	}

	constraints.clear();
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
	}
}

static void init_system(void)
{

	if (runInstance == 0) {
		const double dist = 0.2;
		const Vec2f center(0.0, 0.0);
		const Vec2f offset_x(dist, 0.0);
        const Vec2f offset_y(0.0, dist);


		pVector.push_back(new Particle(center + offset_x));
		pVector.push_back(new Particle(center + offset_x + offset_x));
        pVector.push_back(new Particle(center + offset_x + offset_x + offset_y));	
        pVector.push_back(new Particle(center + offset_x + offset_x + offset_y + offset_x));	
        pVector.push_back(new Particle(center + offset_x + offset_x + offset_x));		

        springForces.push_back(new SpringForce(pVector[0], pVector[1], dist, 50.0, 100.0));

		CircularWireConstraint * circularWireConstraint = new CircularWireConstraint(pVector[0], center, dist);
		circularWireConstraints.push_back(circularWireConstraint);
		constraints.push_back(circularWireConstraint);

		RodConstraint * rodConstraint1 = new RodConstraint(pVector[1], pVector[2], dist);
		rodConstraints.push_back(rodConstraint1);
		constraints.push_back(rodConstraint1);
        RodConstraint * rodConstraint2 = new RodConstraint(pVector[2], pVector[3], dist);
		rodConstraints.push_back(rodConstraint2);
		constraints.push_back(rodConstraint2);
        RodConstraint * rodConstraint3 = new RodConstraint(pVector[3], pVector[4], dist);
		rodConstraints.push_back(rodConstraint3);
		constraints.push_back(rodConstraint3);
        RodConstraint * rodConstraint4 = new RodConstraint(pVector[4], pVector[1], dist);
		rodConstraints.push_back(rodConstraint4);
		constraints.push_back(rodConstraint4);
        
        

    } else if (runInstance == 1) {
        // hair simulation
        dude = true; 
        particle_draw = false; 
        const int hairs = 50;
        const float length = 0.5f; 
        const int segments = 12;
        const float width = 0.4;
        float ks = 120.0f;
        float kd = 1.5f;

        for (int h = 0; h < hairs; h++) {
            // Add particles for each hair 
            for (int y = 0; y < (segments); y++) {
                pVector.push_back(new Particle(Vec2f(
                    width/hairs *h - 0.2, 
                    0.5 + 0.4 * cos(PI/2.4/hairs * h) - (y * length/segments) 
                )));
            }

            // Add springforces between each pair of particles 
            for (int y = 0; y < segments - 1; y++) {
                springForces.push_back(
                    new SpringForce(
                        pVector[h * segments + y],
                        pVector[h * segments + y + 1],
                        length/segments, ks, kd
                    )
                );
            }

            // Add angularspring forces between each triplet of particles 
            for (int y = 0; y < segments - 2; y++) {
                angularSpringForces.push_back(
                    new AngularSpringForce(
                        pVector[h * segments + y],
                        pVector[h * segments + y + 1],
                        pVector[h * segments + y + 2],
                        0.5f, ks, kd
                    )
                );
            }
            
            // Add constraints to the first particle of each hair 
            float radius = 0.001f;
            constraints.push_back(
                new CircularWireConstraint(
                    pVector[h * segments], 
                    pVector[h * segments]->m_ConstructPos + Vec2f(-radius, 0.f), 
                    radius
                )
            );

            const double rail_dist = 0.001;
            const Vec2f rail_start = Vec2f(-1, pVector[h*segments]->m_Position[1] + rail_dist);
            const Vec2f rail_end = Vec2f(1, pVector[h*segments]->m_Position[1] + rail_dist);
            constraints.push_back(new LineConstraint(pVector[h * segments], rail_start, rail_end, rail_dist));

            endpoints.push_back(pVector[((h+1) * segments) - 1]);
        }
    
    
    } else if (runInstance == 2) {
		const int clothSize = 10;
		const double dist = 0.1;
		const Vec2f center(0.0, 0.0);

		
		for (int i = 0; i < clothSize; i++) {
			for (int j = 0; j < clothSize; j++) {
				Vec2f offset((i - (clothSize / 2)) * dist, (j - (clothSize / 2)) * dist);
				pVector.push_back(new Particle(center + offset));
			}
		}

		const Vec2f offset(0, dist / 5);

		CircularWireConstraint * circularWireConstraint = new CircularWireConstraint(pVector[9], pVector[9]->m_ConstructPos + offset, dist / 5);
		circularWireConstraints.push_back(circularWireConstraint);
		constraints.push_back(circularWireConstraint);
		CircularWireConstraint * circularWireConstraint2 = new CircularWireConstraint(pVector[99], pVector[99]->m_ConstructPos + offset, dist / 5);
		circularWireConstraints.push_back(circularWireConstraint2);
		constraints.push_back(circularWireConstraint2);


		for (int i = 0; i < clothSize; i++) {
			for (int j = 0; j < clothSize; j++) {
				if (i != clothSize - 1) {
					springForces.push_back(new SpringForce(pVector[i * clothSize + j], pVector[i * clothSize + j + 10], dist, 50.0, 1.0));
				}

				if (j != clothSize -1) {
					springForces.push_back(new SpringForce(pVector[i * clothSize + j], pVector[i * clothSize + j + 1], dist, 50.0, 1.0));
				}
			}
		}

	}

	if (constraints.size() > 0) {
		constraintSolver = new ConstraintSolver(pVector, constraints);
	}

	if (gravity) {
		gravity_force = new GravityForce(pVector); 
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
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	int size = pVector.size();

	for(int ii=0; ii< size; ii++)
	{
		pVector[ii]->draw();
	}
}

static void draw_forces ( void )
{
	if (springForces.size() > 0) {
		for (SpringForce * springForce: springForces) {
			springForce->draw();
		}
	}

    if (angularSpringForces.size() > 0) {
		for (AngularSpringForce * angularSpringForce: angularSpringForces) {
			angularSpringForce->draw();
		}
	}

}

static void draw_constraints ( void )
{
	if (constraints.size() > 0) {
		for (Constraint * constraint: constraints) {
			constraint->draw();
		}
	}
}

static void draw_dude (void) {
    float head_radius = 0.4f;
    float eye_radius = 0.05f; 
    float eye_offset = 0.1f; 
    Vec2f center = Vec2f(-0.2, 0.5); 
    // head 
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0,1.0,0.0); 
    for (int i=80; i<385; i=i+9)
    {
        float degInRad = i*PI/180;
        glVertex2f(center[0] + cos(degInRad)*head_radius,center[1] + sin(degInRad)*head_radius);
    }
    glEnd();
    // eye 
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0,1.0,0.0); 
    for (int i=0; i<360; i=i+18)
    {
        float degInRad = i*PI/180;
        glVertex2f(center[0] - eye_offset - cos(degInRad) * eye_radius, 
                   center[1] + eye_offset + sin(degInRad) * eye_radius);
    }
    glEnd();
    // mouth 
    glBegin(GL_LINES);
    glColor3f(1.0,0.0,0.0); 
    glVertex2f(center[0] - 0.35, center[1] - 0.2);
    glVertex2f(center[0] - 0.35 + 0.2, center[1] - 0.2);
    glEnd();
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	float x, y;
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       mx /(float)win_x)*N);
	j = (int)(((win_y-my)/(float)win_y)*N);


	if ( i<1 || i>N || j<1 || j>N ) return;

	x = (float) 2 * i / N - 1;
    y = (float) 2 * j / N - 1;

	if ( mouse_down[0] ) {
		Vec2f position(x, y);
		if (!activeMouseParticle) {
			activeMouseParticle = true;
			pVector.push_back(new Particle(position));
            if (runInstance == 1) {
                for (Particle * endpoint: endpoints) {
                    springForces.push_back(new SpringForce(pVector[pVector.size() - 1], endpoint, 0.1 , 50.0, 1.0)); 
                }
            } else {
                springForces.push_back(new SpringForce(pVector[pVector.size() - 1], pVector[pVector.size() - 2], 0.1 , 50.0, 1.0));
            }
		} else {
			pVector[pVector.size() - 1]->m_Position = position;
		}
	}

	if ( mouse_down[2] ) {
	}

	hi = (int)((       hmx /(float)win_x)*N);
	hj = (int)(((win_y-hmy)/(float)win_y)*N);

	
	if( mouse_release[0] ) {
		if (activeMouseParticle) {
			activeMouseParticle = false;
			pVector.pop_back();
            if (runInstance == 1) {
                for (Particle * endpoint: endpoints) {
                    springForces.pop_back(); 
                }
            } else {
                springForces.pop_back(); 
            }
		}
		mouse_release[0] = false;
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->reset();
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

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
		break;
    case ' ':
        dsim = !dsim;
        break;
    case '0':
        free_data();
        runInstance = 0;
        init_system();
        break;
    case '1':
        free_data();
        runInstance = 0;
        init_system();
        break;

    case '2':
        free_data();
        runInstance = 1;
        init_system();
        break;

    case '3':
        free_data();
        runInstance = 2;
        init_system();
        break;
    case 'i':
    case 'I':
        scheme = 0;
        break;
    case 'o':
    case 'O':
        scheme = 1;
        break;
    case 'p':
    case 'P':
        scheme = 2;
        break;
    case 'g':
    case 'G':
        gravity = !gravity;
        break;
    }
   
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
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

static void derivEval() {
	// Reset force_acc
	for (Particle * particle: pVector) {
		particle->clearForce();
	}

	// Calculate all forces working on all particles
	for (SpringForce * springForce: springForces) {
		springForce->calculateForce();
	}

	for (AngularSpringForce * angularSpringForce: angularSpringForces) {
		angularSpringForce->calculateForce();
	}
	gravity_force->calculateGravityForce(); 

	// Calculate all constraint forces working on all particles
	// for (CircularWireConstraint * circularWireConstraint: circularWireConstraints) {
	// 	circularWireConstraint->calculateConstraintForce();
	// }x
	if (constraintSolver) {
		constraintSolver->calculateConstraintForce();
	}
    
    
	// Run a step in the simulation 0 = Euler, 1 = Midpoint, 2 = Runge-Kutta
	simulation_step( pVector, dt, scheme);
}

static void idle_func ( void )
{
	if ( dsim ) {
        // simulation_step( pVector, dt );
        get_from_UI();derivEval();
    } else {
        get_from_UI();remap_GUI();
    }

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

    if (dude) 
        draw_dude(); 
	draw_forces();
	draw_constraints();
    if (particle_draw)
	    draw_particles();

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
	win_id = glutCreateWindow ( "Particletoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

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

	if ( argc == 1 ) {
		N = 64;
		dt = 0.010;		// Simulation speed, default = 0.1
		d = 5.f;
        scheme = 2; 
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
    printf ( "\t Change scenes by pressing the number keys\n" );
    printf ( "\t Change integration scheme by pressing 'i'(=euler), 'o'(=midpoint), 'p'(=rungekutta) \n" );
	printf ( "\t Quit by pressing the 'q' key\n" );
    

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	
	init_system();
	
	win_x = 1024;
	win_y = 1024;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

