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
extern void simulationStep(std::vector<Particle*> pVector, float timeStep, int integrationScheme);

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
static bool dragMode;
static SpringForce * dragSpring = NULL;

static std::vector<Object *> objects;
static std::vector<RigidObject *> rigidObjects;
static std::vector<Particle*> particles;
static FluidForce * fluid_force;
static GravityForce * gravityForce;
static std::vector<SpringForce *> springForces;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
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
		Particle * particle;
		particle = new Particle(center + Vec2f(dist, 0));
		particle->m_Mass = 100;
		particles.push_back(particle);
		particle = new Particle(center + Vec2f(dist, dist));
		particle->m_Mass = 100;
		particles.push_back(particle);
		particle = new Particle(center + Vec2f(0, dist));
		particle->m_Mass = 100;
		particles.push_back(particle);
		particle =new Particle(center);
		particle->m_Mass = 100;
		particles.push_back(particle);
		RigidObject * rigidObject = new RigidObject(particles, N);
		objects.push_back(rigidObject);
		rigidObjects.push_back(rigidObject);

		u[IX(63, 117)] = force * (501 - 93);
		v[IX(63, 117)] = force * (0 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(63, 117)] = force * (501 - 501);
		v[IX(63, 117)] = force * (93 - 93);
		u[IX(62, 117)] = force * (493 - 501);
		v[IX(62, 117)] = force * (93 - 95);
		u[IX(62, 117)] = force * (493 - 493);
		v[IX(62, 117)] = force * (95 - 95);
		u[IX(61, 116)] = force * (480 - 493);
		v[IX(61, 116)] = force * (95 - 97);
		u[IX(61, 116)] = force * (480 - 480);
		v[IX(61, 116)] = force * (97 - 97);
		u[IX(59, 116)] = force * (467 - 480);
		v[IX(59, 116)] = force * (97 - 99);
		u[IX(59, 116)] = force * (467 - 467);
		v[IX(59, 116)] = force * (99 - 99);
		u[IX(58, 116)] = force * (459 - 467);
		v[IX(58, 116)] = force * (99 - 101);
		u[IX(58, 116)] = force * (459 - 459);
		v[IX(58, 116)] = force * (101 - 101);
		u[IX(57, 116)] = force * (453 - 459);
		v[IX(57, 116)] = force * (101 - 103);
		u[IX(57, 116)] = force * (453 - 453);
		v[IX(57, 116)] = force * (103 - 103);
		u[IX(57, 115)] = force * (450 - 453);
		v[IX(57, 115)] = force * (103 - 105);
		u[IX(57, 115)] = force * (450 - 450);
		v[IX(57, 115)] = force * (105 - 105);
		u[IX(56, 115)] = force * (444 - 450);
		v[IX(56, 115)] = force * (105 - 109);
		u[IX(56, 115)] = force * (444 - 444);
		v[IX(56, 115)] = force * (109 - 109);
		u[IX(55, 114)] = force * (435 - 444);
		v[IX(55, 114)] = force * (109 - 115);
		u[IX(55, 114)] = force * (435 - 435);
		v[IX(55, 114)] = force * (115 - 115);
		u[IX(55, 114)] = force * (432 - 435);
		v[IX(55, 114)] = force * (115 - 119);
		u[IX(55, 114)] = force * (432 - 432);
		v[IX(55, 114)] = force * (119 - 119);
		u[IX(54, 113)] = force * (430 - 432);
		v[IX(54, 113)] = force * (119 - 121);
		u[IX(54, 113)] = force * (430 - 430);
		v[IX(54, 113)] = force * (121 - 121);
		u[IX(54, 113)] = force * (429 - 430);
		v[IX(54, 113)] = force * (121 - 123);
		u[IX(54, 113)] = force * (429 - 429);
		v[IX(54, 113)] = force * (123 - 123);
		u[IX(54, 112)] = force * (425 - 429);
		v[IX(54, 112)] = force * (123 - 129);
		u[IX(54, 112)] = force * (425 - 425);
		v[IX(54, 112)] = force * (129 - 129);
		u[IX(53, 110)] = force * (417 - 425);
		v[IX(53, 110)] = force * (129 - 151);
		u[IX(53, 110)] = force * (417 - 417);
		v[IX(53, 110)] = force * (151 - 151);
		u[IX(52, 107)] = force * (410 - 417);
		v[IX(52, 107)] = force * (151 - 170);
		u[IX(52, 107)] = force * (410 - 410);
		v[IX(52, 107)] = force * (170 - 170);
		u[IX(51, 106)] = force * (406 - 410);
		v[IX(51, 106)] = force * (170 - 184);
		u[IX(51, 106)] = force * (406 - 406);
		v[IX(51, 106)] = force * (184 - 184);
		u[IX(51, 106)] = force * (406 - 406);
		v[IX(51, 106)] = force * (184 - 184);
		u[IX(51, 103)] = force * (405 - 406);
		v[IX(51, 103)] = force * (184 - 201);
		u[IX(51, 101)] = force * (405 - 405);
		v[IX(51, 101)] = force * (201 - 220);
		u[IX(51, 101)] = force * (405 - 405);
		v[IX(51, 101)] = force * (220 - 220);
		u[IX(51, 101)] = force * (405 - 405);
		v[IX(51, 101)] = force * (220 - 220);
		u[IX(51, 99)] = force * (406 - 405);
		v[IX(51, 99)] = force * (220 - 237);
		u[IX(51, 99)] = force * (406 - 406);
		v[IX(51, 99)] = force * (237 - 237);
		u[IX(51, 97)] = force * (407 - 406);
		v[IX(51, 97)] = force * (237 - 254);
		u[IX(51, 97)] = force * (407 - 407);
		v[IX(51, 97)] = force * (254 - 254);
		u[IX(52, 94)] = force * (411 - 407);
		v[IX(52, 94)] = force * (254 - 275);
		u[IX(52, 94)] = force * (411 - 411);
		v[IX(52, 94)] = force * (275 - 275);
		u[IX(52, 92)] = force * (414 - 411);
		v[IX(52, 92)] = force * (275 - 296);
		u[IX(52, 92)] = force * (414 - 414);
		v[IX(52, 92)] = force * (296 - 296);
		u[IX(53, 90)] = force * (419 - 414);
		v[IX(53, 90)] = force * (296 - 312);
		u[IX(53, 90)] = force * (419 - 419);
		v[IX(53, 90)] = force * (312 - 312);
		u[IX(54, 88)] = force * (424 - 419);
		v[IX(54, 88)] = force * (312 - 327);
		u[IX(54, 88)] = force * (424 - 424);
		v[IX(54, 88)] = force * (327 - 327);
		u[IX(54, 86)] = force * (428 - 424);
		v[IX(54, 86)] = force * (327 - 341);
		u[IX(54, 86)] = force * (428 - 428);
		v[IX(54, 86)] = force * (341 - 341);
		u[IX(54, 86)] = force * (428 - 428);
		v[IX(54, 86)] = force * (341 - 341);
		u[IX(55, 84)] = force * (434 - 428);
		v[IX(55, 84)] = force * (341 - 353);
		u[IX(55, 83)] = force * (439 - 434);
		v[IX(55, 83)] = force * (353 - 363);
		u[IX(55, 83)] = force * (439 - 439);
		v[IX(55, 83)] = force * (363 - 363);
		u[IX(55, 83)] = force * (439 - 439);
		v[IX(55, 83)] = force * (363 - 363);
		u[IX(56, 82)] = force * (444 - 439);
		v[IX(56, 82)] = force * (363 - 371);
		u[IX(57, 81)] = force * (448 - 444);
		v[IX(57, 81)] = force * (371 - 377);
		u[IX(57, 81)] = force * (448 - 448);
		v[IX(57, 81)] = force * (377 - 377);
		u[IX(57, 81)] = force * (448 - 448);
		v[IX(57, 81)] = force * (377 - 377);
		u[IX(57, 81)] = force * (452 - 448);
		v[IX(57, 81)] = force * (377 - 381);
		u[IX(57, 81)] = force * (452 - 452);
		v[IX(57, 81)] = force * (381 - 381);
		u[IX(58, 80)] = force * (457 - 452);
		v[IX(58, 80)] = force * (381 - 386);
		u[IX(58, 80)] = force * (457 - 457);
		v[IX(58, 80)] = force * (386 - 386);
		u[IX(58, 80)] = force * (463 - 457);
		v[IX(58, 80)] = force * (386 - 391);
		u[IX(60, 79)] = force * (472 - 463);
		v[IX(60, 79)] = force * (391 - 396);
		u[IX(61, 79)] = force * (483 - 472);
		v[IX(61, 79)] = force * (396 - 400);
		u[IX(61, 79)] = force * (483 - 483);
		v[IX(61, 79)] = force * (400 - 400);
		u[IX(63, 78)] = force * (497 - 483);
		v[IX(63, 78)] = force * (400 - 405);
		u[IX(64, 78)] = force * (510 - 497);
		v[IX(64, 78)] = force * (405 - 408);
		u[IX(64, 78)] = force * (510 - 510);
		v[IX(64, 78)] = force * (408 - 408);
		u[IX(67, 77)] = force * (528 - 510);
		v[IX(67, 77)] = force * (408 - 412);
		u[IX(69, 77)] = force * (545 - 528);
		v[IX(69, 77)] = force * (412 - 413);
		u[IX(72, 77)] = force * (573 - 545);
		v[IX(72, 77)] = force * (413 - 414);
		u[IX(75, 77)] = force * (597 - 573);
		v[IX(75, 77)] = force * (414 - 414);
		u[IX(75, 77)] = force * (597 - 597);
		v[IX(75, 77)] = force * (414 - 414);
		u[IX(78, 77)] = force * (622 - 597);
		v[IX(78, 77)] = force * (414 - 414);
		u[IX(82, 77)] = force * (651 - 622);
		v[IX(82, 77)] = force * (414 - 409);
		u[IX(82, 77)] = force * (651 - 651);
		v[IX(82, 77)] = force * (409 - 409);
		u[IX(86, 78)] = force * (680 - 651);
		v[IX(86, 78)] = force * (409 - 404);
		u[IX(86, 78)] = force * (680 - 680);
		v[IX(86, 78)] = force * (404 - 404);
		u[IX(89, 79)] = force * (704 - 680);
		v[IX(89, 79)] = force * (404 - 398);
		u[IX(92, 80)] = force * (729 - 704);
		v[IX(92, 80)] = force * (398 - 391);
		u[IX(92, 80)] = force * (729 - 729);
		v[IX(92, 80)] = force * (391 - 391);
		u[IX(94, 81)] = force * (746 - 729);
		v[IX(94, 81)] = force * (391 - 384);
		u[IX(95, 81)] = force * (757 - 746);
		v[IX(95, 81)] = force * (384 - 378);
		u[IX(97, 82)] = force * (768 - 757);
		v[IX(97, 82)] = force * (378 - 374);
		u[IX(97, 82)] = force * (768 - 768);
		v[IX(97, 82)] = force * (374 - 374);
		u[IX(98, 82)] = force * (776 - 768);
		v[IX(98, 82)] = force * (374 - 369);
		u[IX(98, 82)] = force * (776 - 776);
		v[IX(98, 82)] = force * (369 - 369);
		u[IX(98, 83)] = force * (782 - 776);
		v[IX(98, 83)] = force * (369 - 364);
		u[IX(98, 83)] = force * (782 - 782);
		v[IX(98, 83)] = force * (364 - 364);
		u[IX(99, 84)] = force * (787 - 782);
		v[IX(99, 84)] = force * (364 - 359);
		u[IX(99, 84)] = force * (787 - 787);
		v[IX(99, 84)] = force * (359 - 359);
		u[IX(99, 84)] = force * (791 - 787);
		v[IX(99, 84)] = force * (359 - 354);
		u[IX(99, 84)] = force * (791 - 791);
		v[IX(99, 84)] = force * (354 - 354);
		u[IX(99, 84)] = force * (791 - 791);
		v[IX(99, 84)] = force * (354 - 354);
		u[IX(100, 85)] = force * (795 - 791);
		v[IX(100, 85)] = force * (354 - 347);
		u[IX(100, 86)] = force * (799 - 795);
		v[IX(100, 86)] = force * (347 - 338);
		u[IX(100, 86)] = force * (799 - 799);
		v[IX(100, 86)] = force * (338 - 338);
		u[IX(101, 88)] = force * (802 - 799);
		v[IX(101, 88)] = force * (338 - 327);
		u[IX(101, 88)] = force * (802 - 802);
		v[IX(101, 88)] = force * (327 - 327);
		u[IX(101, 89)] = force * (805 - 802);
		v[IX(101, 89)] = force * (327 - 314);
		u[IX(101, 89)] = force * (805 - 805);
		v[IX(101, 89)] = force * (314 - 314);
		u[IX(101, 91)] = force * (805 - 805);
		v[IX(101, 91)] = force * (314 - 298);
		u[IX(101, 91)] = force * (805 - 805);
		v[IX(101, 91)] = force * (298 - 298);
		u[IX(101, 94)] = force * (803 - 805);
		v[IX(101, 94)] = force * (298 - 280);
		u[IX(101, 94)] = force * (803 - 803);
		v[IX(101, 94)] = force * (280 - 280);
		u[IX(100, 97)] = force * (795 - 803);
		v[IX(100, 97)] = force * (280 - 255);
		u[IX(100, 97)] = force * (795 - 795);
		v[IX(100, 97)] = force * (255 - 255);
		u[IX(99, 100)] = force * (785 - 795);
		v[IX(99, 100)] = force * (255 - 225);
		u[IX(99, 100)] = force * (785 - 785);
		v[IX(99, 100)] = force * (225 - 225);
		u[IX(97, 103)] = force * (773 - 785);
		v[IX(97, 103)] = force * (225 - 201);
		u[IX(97, 103)] = force * (773 - 773);
		v[IX(97, 103)] = force * (201 - 201);
		u[IX(95, 106)] = force * (759 - 773);
		v[IX(95, 106)] = force * (201 - 179);
		u[IX(95, 106)] = force * (759 - 759);
		v[IX(95, 106)] = force * (179 - 179);
		u[IX(94, 108)] = force * (747 - 759);
		v[IX(94, 108)] = force * (179 - 161);
		u[IX(94, 108)] = force * (747 - 747);
		v[IX(94, 108)] = force * (161 - 161);
		u[IX(93, 110)] = force * (739 - 747);
		v[IX(93, 110)] = force * (161 - 149);
		u[IX(92, 111)] = force * (731 - 739);
		v[IX(92, 111)] = force * (149 - 139);
		u[IX(92, 111)] = force * (731 - 731);
		v[IX(92, 111)] = force * (139 - 139);
		u[IX(92, 111)] = force * (731 - 731);
		v[IX(92, 111)] = force * (139 - 139);
		u[IX(91, 112)] = force * (723 - 731);
		v[IX(91, 112)] = force * (139 - 131);
		u[IX(91, 112)] = force * (723 - 723);
		v[IX(91, 112)] = force * (131 - 131);
		u[IX(89, 113)] = force * (711 - 723);
		v[IX(89, 113)] = force * (131 - 121);
		u[IX(89, 113)] = force * (711 - 711);
		v[IX(89, 113)] = force * (121 - 121);
		u[IX(88, 114)] = force * (698 - 711);
		v[IX(88, 114)] = force * (121 - 113);
		u[IX(88, 114)] = force * (698 - 698);
		v[IX(88, 114)] = force * (113 - 113);
		u[IX(86, 116)] = force * (684 - 698);
		v[IX(86, 116)] = force * (113 - 104);
		u[IX(86, 116)] = force * (684 - 684);
		v[IX(86, 116)] = force * (104 - 104);
		u[IX(84, 117)] = force * (669 - 684);
		v[IX(84, 117)] = force * (104 - 96);
		u[IX(84, 117)] = force * (669 - 669);
		v[IX(84, 117)] = force * (96 - 96);
		u[IX(84, 117)] = force * (669 - 669);
		v[IX(84, 117)] = force * (96 - 96);
		u[IX(82, 117)] = force * (651 - 669);
		v[IX(82, 117)] = force * (96 - 90);
		u[IX(80, 118)] = force * (634 - 651);
		v[IX(80, 118)] = force * (90 - 83);
		u[IX(80, 118)] = force * (634 - 634);
		v[IX(80, 118)] = force * (83 - 83);
		u[IX(80, 118)] = force * (634 - 634);
		v[IX(80, 118)] = force * (83 - 83);
		u[IX(77, 119)] = force * (615 - 634);
		v[IX(77, 119)] = force * (83 - 78);
		u[IX(75, 119)] = force * (599 - 615);
		v[IX(75, 119)] = force * (78 - 74);
		u[IX(75, 119)] = force * (599 - 599);
		v[IX(75, 119)] = force * (74 - 74);
		u[IX(75, 119)] = force * (599 - 599);
		v[IX(75, 119)] = force * (74 - 74);
		u[IX(73, 120)] = force * (583 - 599);
		v[IX(73, 120)] = force * (74 - 70);
		u[IX(71, 120)] = force * (564 - 583);
		v[IX(71, 120)] = force * (70 - 65);
		u[IX(71, 120)] = force * (564 - 564);
		v[IX(71, 120)] = force * (65 - 65);
		u[IX(69, 121)] = force * (551 - 564);
		v[IX(69, 121)] = force * (65 - 63);
		u[IX(68, 121)] = force * (537 - 551);
		v[IX(68, 121)] = force * (63 - 58);
		u[IX(66, 122)] = force * (526 - 537);
		v[IX(66, 122)] = force * (58 - 55);
		u[IX(66, 122)] = force * (526 - 526);
		v[IX(66, 122)] = force * (55 - 55);
		u[IX(65, 122)] = force * (517 - 526);
		v[IX(65, 122)] = force * (55 - 53);
		u[IX(65, 122)] = force * (512 - 517);
		v[IX(65, 122)] = force * (53 - 52);
		u[IX(64, 122)] = force * (509 - 512);
		v[IX(64, 122)] = force * (52 - 51);
		u[IX(64, 122)] = force * (505 - 509);
		v[IX(64, 122)] = force * (51 - 51);
		u[IX(64, 122)] = force * (505 - 505);
		v[IX(64, 122)] = force * (51 - 51);
		u[IX(64, 122)] = force * (505 - 505);
		v[IX(64, 122)] = force * (51 - 51);
		u[IX(63, 122)] = force * (500 - 505);
		v[IX(63, 122)] = force * (51 - 51);
		u[IX(63, 122)] = force * (498 - 500);
		v[IX(63, 122)] = force * (51 - 51);
		u[IX(63, 122)] = force * (496 - 498);
		v[IX(63, 122)] = force * (51 - 52);
		u[IX(63, 122)] = force * (496 - 496);
		v[IX(63, 122)] = force * (52 - 52);
		u[IX(62, 122)] = force * (493 - 496);
		v[IX(62, 122)] = force * (52 - 54);
		u[IX(62, 122)] = force * (493 - 493);
		v[IX(62, 122)] = force * (54 - 54);
		u[IX(62, 122)] = force * (491 - 493);
		v[IX(62, 122)] = force * (54 - 55);
		u[IX(62, 122)] = force * (491 - 491);
		v[IX(62, 122)] = force * (55 - 55);
		u[IX(62, 121)] = force * (489 - 491);
		v[IX(62, 121)] = force * (55 - 57);
		u[IX(62, 121)] = force * (488 - 489);
		v[IX(62, 121)] = force * (57 - 58);
		u[IX(62, 121)] = force * (488 - 488);
		v[IX(62, 121)] = force * (58 - 58);
		u[IX(61, 121)] = force * (487 - 488);
		v[IX(61, 121)] = force * (58 - 58);
		u[IX(61, 121)] = force * (487 - 487);
		v[IX(61, 121)] = force * (58 - 58);

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

	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] ) return;

	i = (int)((       mx /(float)win_x)*N+1);
	j = (int)(((win_y-my)/(float)win_y)*N+1);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {
		if (dragMode && rigidObjects.size() > 0) {
			float x = (float)  i / N; float y = (float)  j / N;

			if (dragSpring == NULL) {
				Particle * mouseParticle = new Particle(Vec2f(x, y));
				mouseParticle->m_Mass = 0;


				RigidObject * rb; 
				float dist = INFINITY;
				for (RigidObject * rbCheck: rigidObjects) {
					float rbDist = sqrt(pow(mouseParticle->m_Position[0] - rbCheck->x[0], 2)
									+ pow(mouseParticle->m_Position[1] - rbCheck->x[1], 2) );
					if (rbDist < dist) {
						dist = rbDist;
						rb = rbCheck;
					}
				}

				Particle * particle;
				dist = INFINITY;
				for (Particle * rbParticle: rb->particles) {
					float particleDist = sqrt(pow(mouseParticle->m_Position[0] - (rbParticle->m_Position[0] + rb->x[0]), 2)
									+ pow(mouseParticle->m_Position[1] - (rbParticle->m_Position[1] + rb->x[1]), 2) );
				
					if (particleDist < dist) {
						dist = particleDist;
						particle = rbParticle;
					}
				}
				printf("dist: %d", dist);
				dragSpring = new SpringForce(particle, mouseParticle ,dist, 10, 10);
				dragSpring->rb = rb;
				springForces.push_back(dragSpring);
			} else {
				Particle * mouseParticle = dragSpring->getParticles()[1];
				mouseParticle->m_Position = Vec2f(x, y);
			}
		} else {
			u[IX(i,j)] = force * (mx-omx);
			printf("u[IX(%d, %d)] = force * (%d - %d);\n", i, j , mx, omx);
			v[IX(i,j)] = force * (omy-my);
			printf("v[IX(%d, %d)] = force * (%d - %d);\n", i, j , omy, my);

		}

	}

	if (mouse_release[0]) {
		if (dragSpring != NULL) {
			springForces.pop_back();
			dragSpring = NULL;
		}

		mouse_release[0] = false;
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
		case 'd':
		case 'D':
			dragMode = !dragMode;
			break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if (mouse_down[button]) mouse_release[button] = state = GLUT_UP;
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

static void applySpringForces() {
	for (SpringForce * spring_force: springForces) {
		spring_force->calculateForce(false);
	}
}

static void idle_func ( void )
{
	clearForces();

	get_from_UI ( dens_prev, u_prev, v_prev );
	vel_step ( N, u, v, u_prev, v_prev, uVort, vVort, visc, dt, eps, vorticity_confinement );
	dens_step ( N, dens, dens_prev, u, v, diff, dt );	

    if ( dsim ) {
		applySpringForces();
		fluid_force->calculateForce(N, dt, dens, u, v, u_prev, v_prev);

		// for (RigidObject * rb: rigidObjects) {
		// 	for (Particle * particle: rb->particles) {
		// 	int particleCoords = IX((int)((rb->x[0] + particle->m_Position[0]) * N),
		// 			(int)((rb->x[1] + particle->m_Position[1]) * N));

        //     u[particleCoords] += rb->v[0] * 50;
        //     v[particleCoords] += rb->v[1] * 50;
		// 	}
		// }

		simulationStep(particles, dt, integrationScheme);
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
		dt = 0.03f;
		diff = 0.0f;
		visc = 0.0f;
		force = 5.0f;
		source = 100.0f;
		eps = 10;
		vorticity_confinement = true;
		dragMode = false;
		dvel = true;
		dsim = false;

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

	// dvel = 0;
    // dsim = 0;

	if ( !allocate_data () ) exit ( 1 );
	clear_data ();

    init_system();

	win_x = 1024;
	win_y = 1024;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}