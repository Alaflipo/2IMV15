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
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>

#include <Eigen/Dense>
using namespace Eigen;
/* macros */

/* external definitions (from solver) */
extern void simulation_step(std::vector<Particle*> pVector, float dt, int scheme);

/* global variables */

static int runIdx = 0;

static int N;
static float dt, d;
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
static std::vector<CircularWireConstraint *> circularWireConstraints;
static std::vector<RodConstraint *> rodConstraints;

static std::vector<Constraint *> constraints;
static ConstraintSolver * constraintSolver;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	pVector.clear();

	springForces.clear();
	circularWireConstraints.clear();
	rodConstraints.clear();
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
	const int runInstance = 1;
	const bool gravity = true;

	if (runInstance == 0) {
		const double dist = 0.2;
		const Vec2f center(0.0, 0.0);
		const Vec2f offset(dist, 0.0);

		pVector.push_back(new Particle(center + offset));
		pVector.push_back(new Particle(center + offset + offset));
		pVector.push_back(new Particle(center + offset + offset + offset));

		CircularWireConstraint * circularWireConstraint = new CircularWireConstraint(pVector[0], center, dist);
		circularWireConstraints.push_back(circularWireConstraint);
		constraints.push_back(circularWireConstraint);

		RodConstraint * rodConstraint = new RodConstraint(pVector[0], pVector[1], dist);
		rodConstraints.push_back(rodConstraint);
		constraints.push_back(rodConstraint);
	}
	else if (runInstance == 1) {
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
					springForces.push_back(new SpringForce(pVector[i * clothSize + j], pVector[i * clothSize + j + 10], dist, 200.0, 10.0));
				}

				if (j != clothSize -1) {
					springForces.push_back(new SpringForce(pVector[i * clothSize + j], pVector[i * clothSize + j + 1], dist, 200.0, 10.0));
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

void implicitEulerStep() 
{
	int dimensions = 2;
	
	for (SpringForce * springForce: springForces) {
		std::vector<Particle *> particles = springForce->getParticles();
		Vec2f dxVec = ((particles[0]->m_Position) - (particles[1]->m_Position));
		VectorXf dx = VectorXf::Zero(dimensions);
		dx[0] = ((particles[0]->m_Position) - (particles[1]->m_Position))[0];
		dx[1] = ((particles[0]->m_Position) - (particles[1]->m_Position))[1];
		MatrixXf I = MatrixXf::Identity(dimensions, dimensions);
		MatrixXf dxtdx = MatrixXf::Zero(dx.size(), dx.size());
		dxtdx = (dx * (dx.transpose()));

		double l = sqrt(dxVec * dxVec);
		if (l != 0) {
			l = 1 / l;
		}

		dxtdx = dxtdx * (l * l);
		springForce->Jx = (dxtdx + ((I - dxtdx) * (1 - (springForce->m_dist * l)))) * (springForce->m_ks);
		springForce->Jv = MatrixXf::Identity(dimensions, dimensions);
		springForce->Jv *= springForce->m_kd;
	}
}

static void MultiplyDfDx(std::vector<VectorXf>  src, std::vector<VectorXf> dst) 
{
	// std::cout<< "UWUWUWUWUWUWUW" << std::endl;
	for (int i = 0; i < pVector.size(); i++) {
		// std::cout<< "0" << std::endl;
		VectorXf tmp = VectorXf::Zero(2);
		dst[i] = tmp;
	}
	// std::cout<< "1" << std::endl;
	for (SpringForce* springForce: springForces) {
		VectorXf temp = VectorXf::Zero(2);
		std::vector<Particle*> particles = springForce->getParticles();
		std::vector<int> particleIndices = {0, 0};
		// std::cout<< "2" << std::endl;
		for (int i = 0; i < pVector.size(); i++) {
			if (pVector[i] == particles[0]) {
				particleIndices[0] = i;
			} else if (pVector[i] == particles[1]) {
				particleIndices[1] = i;
			}
		}
		// std::cout<< "3" << std::endl;
		// std::cout << "cols2: " << src[particleIndices[0]].cols() << " " << src[particleIndices[1]].cols() << std::endl;
		// std::cout << (src[particleIndices[0]] - src[particleIndices[1]]).cols();
		// std::cout << "particle index 1: " << particleIndices[1] << std::endl;
		// for (auto item: src[particleIndices[1]]) {
		// 	std::cout << item << "  ";
		// }
		// std::cout << "rows2: " << src[particleIndices[0]].rows() << " " << src[particleIndices[1]].rows() << std::endl;
		// std::cout << (src[particleIndices[0]] - src[particleIndices[1]]).rows();
		// std::cout << "cols: " << springForce->Jx.cols() << std::endl;
		temp = springForce->Jx * (src[particleIndices[0]] - src[particleIndices[1]]);
		// std::cout << "jx[0]: " << springForce->Jx << std::endl;
		// std::cout << "src1: " << src[particleIndices[0]] << "  src2: " << src[particleIndices[1]] << std::endl;
		// std::cout << "src substraction: " << std::endl << (src[particleIndices[0]] - src[particleIndices[1]]) << std::endl;
		// std::cout << "temp: " << std::endl << temp << std::endl;
		dst[particleIndices[0]] -= temp;
		// std::cout << "dst:[" << particleIndices[0] << "]: " << std::endl << dst[particleIndices[0]] << std::endl;
		// std::cout << "	dst" << particleIndices[0] << ": " << dst[particleIndices[0]][0] << "  " << dst[particleIndices[0]][1] << std::endl;
		dst[particleIndices[1]] += temp;
		// std::cout << "	dst" << particleIndices[1] << ": " << dst[particleIndices[1]][0] << "  " << dst[particleIndices[1]][1] << std::endl;
	}
}

static void MultiplyDfDv(std::vector<VectorXf> src, std::vector<VectorXf> dst) 
{
	for (int i = 0; i < pVector.size(); i++) {
		VectorXf tmp = VectorXf::Zero(2);
		dst[i] = tmp;
		// std::cout << "dst" << i << ": " << dst[i][0] << "  " << dst[i][1] << std::endl;
	}

	for (SpringForce* springForce: springForces) {
		VectorXf temp = VectorXf::Zero(2);
		std::vector<Particle*> particles = springForce->getParticles();
		std::vector<int> particleIndices = {0, 0};
		for (int i = 0; i < pVector.size(); i++) {
			if (pVector[i] == particles[0]) {
				particleIndices[0] = i;
			} else if (pVector[i] == particles[1]) {
				particleIndices[1] = i;
			}
		}
		temp = springForce->Jv * (src[particleIndices[0]] - src[particleIndices[1]]);
		// std::cout << "src substraction: " << (src[particleIndices[0]] - src[particleIndices[1]])[0] << "  " << (src[particleIndices[0]] - src[particleIndices[1]])[1] << std::endl;
		// std::cout << "temp: " << temp[0] << "  " << temp[1] << std::endl;
		dst[particleIndices[0]] -= temp;

		// std::cout << "	dst" << particleIndices[0] << ": " << dst[particleIndices[0]][0] << "  " << dst[particleIndices[0]][1] << std::endl;
		dst[particleIndices[1]] += temp;
		// std::cout << "	dst" << particleIndices[1] << ": " << dst[particleIndices[1]][0] << "  " << dst[particleIndices[1]][1] << std::endl;
	}
}

void solveLinearSystem() 
{
	implicitEulerStep();

	MatrixXf A = MatrixXf::Zero(2 * pVector.size(), 2 * pVector.size());
	MatrixXf M = MatrixXf::Zero(2 * pVector.size(), 2 * pVector.size());
	for (int i = 0; i < pVector.size(); i++) {
		for (int j = 0; j < 2; j++) {
			M(i * 2 + j, i * 2 + j) = pVector[i]->m_Mass;
		}
	}

	VectorXf b = VectorXf::Zero(2 * pVector.size());
	VectorXf f0 = VectorXf::Zero(2 * pVector.size());
	std::vector<VectorXf> x0;
	std::vector<VectorXf> v0; std::vector<VectorXf> dfdxv0(pVector.size(), VectorXf()); 
	std::vector<VectorXf> dtVec; std::vector<VectorXf> dfdvdt(pVector.size(), VectorXf());
	std::vector<VectorXf> dt2Vec; std::vector<VectorXf> dfdxdt2(pVector.size(), VectorXf());

	for (int i = 0; i < pVector.size(); i++) {
		VectorXf tmp = VectorXf::Zero(2);
		tmp[0] = pVector[i]->get_state()[0][0];
		tmp[1] = pVector[i]->get_state()[0][1];
		x0.push_back(tmp);
		// std::cout << "We take those" << std::endl;
		VectorXf tmp2 = VectorXf::Zero(2);
		tmp2[0] = pVector[i]->get_state()[1][0];
		tmp2[1] = pVector[i]->get_state()[1][1];
		v0.push_back(tmp2);
		f0[i * 2] = pVector[i]->derivEval()[1][0];
		f0[i * 2 + 1] = pVector[i]->derivEval()[1][1];
		VectorXf tmp3 = VectorXf::Zero(2);
		tmp3[0] = dt;
		tmp3[1] = dt;
		dtVec.push_back(tmp3);
		VectorXf tmp4 = VectorXf::Zero(2);
		tmp4[0] = dt * dt;
		tmp4[1] = dt * dt;
		dt2Vec.push_back(tmp4);
	}

	// int idx = 0;
	// for (VectorXf veci: v0) {
	// 	std::cout << "v0[" << idx << "]: " << std::endl << veci << std::endl;
	// 	idx += 1;
	// }
	
	MultiplyDfDx(v0, dfdxv0);
	MultiplyDfDv(dtVec, dfdvdt);
	MultiplyDfDx(dt2Vec, dfdxdt2);

	// std::cout << "dfdxv0 size: " << dfdxv0.size() << std::endl;
	// std::cout << "dfdxv0[0] size: " << dfdxv0[0][0] << dfdxv0[0][1] << std::endl;
	// std::cout << "dfdvdtVec size: " << dfdvdtVec.size() << std::endl;
	// std::cout << "dfdxdt2Vec size: " << dfdxdt2Vec.size() << std::endl;

	VectorXf dfdxv0Vec = VectorXf::Zero(2 * pVector.size());
	MatrixXf dfdvdtMat = MatrixXf::Zero(2 * pVector.size(), 2 * pVector.size());
	MatrixXf dfdxdt2Mat = MatrixXf::Zero(2 * pVector.size(), 2 * pVector.size());

	// std::cout << "dfdxv0Vec: " << std::endl << dfdxv0[0] << std::endl;
	// if (dfdxv0[0] == VectorXf()) {
	// 	std::cout << "NULL" << std::endl;
	// }
	// std::cout << "dfdxv0Vec: " << std::endl << dfdxv0Vec[99] << std::endl;
	// std::cout << "dfdxv0Vec: " << std::endl << dfdxv0Vec[101] << std::endl;

	// std::cout << "dfdxv0 size: " << dfdxv0.size() << std::endl;
	for (int i = 0; i < dfdxv0.size(); i++) {
		for (int j = 0; j < 2; j++) {
			// std::cout << "i: " << i << "  j: " << j << std::endl;
			if (dfdxv0[i] == VectorXf()) {
				dfdxv0Vec[i * 2 + j] = 0;
			} else {
				dfdxv0Vec[i * 2 + j] = dfdxv0[i][j];
			}

			if (dfdvdt[i] == VectorXf()) {
				dfdvdtMat(i * 2 + j, i * 2 + j) = 0;
			} else {
				dfdvdtMat(i * 2 + j, i * 2 + j) = dfdvdt[i][j];
			}

			if (dfdxdt2[i] == VectorXf()) {
				dfdxdt2Mat(i * 2 + j, i * 2 + j) = 0;
			} else {
				dfdxdt2Mat(i * 2 + j, i * 2 + j) = dfdxdt2[i][j];
			}			
		}
	}

	b = (dt *(f0 + (dt * dfdxv0Vec)));
	A = M - (dfdvdtMat) - (dfdxdt2Mat);

	ConjugateGradient<MatrixXf, Lower|Upper> cg;
	cg.compute(A);
	Eigen::Solve<Eigen::ConjugateGradient<Eigen::MatrixXf, 3>, Eigen::VectorXf> dv = cg.solve(b);
	std::vector<VectorXf> dx;

	for (int i = 0; i < 100; i++) {
		VectorXf tmpVec = VectorXf::Zero(2);
		tmpVec[0] = dv[i * 2];
		tmpVec[1] = dv[i * 2 + 1];
		dx.push_back(tmpVec);
	}

	for (int i = 0; i < v0.size(); i++) {
		dx[i] += v0[i];
		dx[i] *= dt;
		
	}
	std::cout << "dx[" << 1 << "]: " << dx[1] << std::endl;
	std::cout << "x0[1]" << x0[1] << std::endl;

	auto tank = pVector[1]->get_state();
	std::cout << "old: pos: "<< tank[0] << "   vel: " << tank[1] << std::endl;

	for (int i = 0; i < pVector.size(); i++) {
		Vec2f xNew = Vec2f(x0[i][0] + dx[i][0], x0[i][1] + dx[i][1]);
		Vec2f vNew = Vec2f(v0[i][0] + dv[i * 2], v0[i][1] + dv[i * 2 + 1]);
		pVector[i]->set_state(xNew, vNew);
		pVector[i]->clearForce();
	}  

	tank = pVector[1]->get_state();
	std::cout << "new: pos: "<< tank[0] << "   vel: " << tank[1] << std::endl;

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

}

static void draw_constraints ( void )
{
	if (constraints.size() > 0) {
		for (Constraint * constraint: constraints) {
			constraint->draw();
		}
	}
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

			Particle* closestParticle = NULL;
			float minDistance = 100000;

			for (int i = 0; i < pVector.size() -1; i++) {
				Particle* part = pVector[i];
				Vec2f diff = part->m_Position - position;
				float dist = sqrt(diff * diff);
				if (dist < minDistance) {
					minDistance = dist;
					closestParticle = part;
				};
			}

			if (closestParticle) {
				activeMouseParticle = true;
				pVector.push_back(new Particle(position));
				springForces.push_back(new SpringForce(pVector[pVector.size() - 1], closestParticle, minDistance, 50.0, 1.0));
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
			springForces.pop_back();
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
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
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
	gravity_force->calculateGravityForce(); 

	// Calculate all constraint forces working on all particles
	// for (CircularWireConstraint * circularWireConstraint: circularWireConstraints) {
	// 	circularWireConstraint->calculateConstraintForce();
	// }
	if (constraintSolver) {
		constraintSolver->calculateConstraintForce();
	}
    
    
	// Run a step in the simulation 0 = Euler, 1 = Midpoint, 2 = Runge-Kutta
	// simulation_step( pVector, dt, 2);

	// simulation_step( pVector, dt, 2);
	solveLinearSystem();

	// if (runIdx == 0) {
	// 	simulation_step( pVector, dt, 0);
	// 	runIdx += 1;
	// } else{
	// 	solveLinearSystem();
	// }
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

	draw_forces();
	draw_constraints();
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
		dt = 0.03;		// Simulation speed, default = 0.1
		d = 5.f;
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
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	
	init_system();
	
	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

