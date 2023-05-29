#include "CircularWireConstraint.h"
#if defined(__CYGWIN__) || defined(WIN32)
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <gfx/mat2.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle* p, const Vec2f & center, const double radius) :
	m_p(p), m_center(center), m_radius(radius) {
}

CircularWireConstraint::~CircularWireConstraint(void) {}

std::vector<Particle*> CircularWireConstraint::getParticles()
{
    std::vector<Particle*> particles;
    particles.push_back(m_p);
    return particles;
}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

float CircularWireConstraint::getConstraint()
{
    return pow((m_p->m_Position - m_center)[0], 2) + pow((m_p->m_Position - m_center)[1], 2) - pow(m_radius, 2);
}

float CircularWireConstraint::getConstraintDerivative()
{
    return 2 * (m_p->m_Position - m_center) * m_p->m_Velocity;
}

std::vector<Vec2f> CircularWireConstraint::getJacobian()
{
    return std::vector<Vec2f> {(m_p->m_Position - m_center) * 2};
}

std::vector<Vec2f> CircularWireConstraint::getJacobianDerivative()
{
    return std::vector<Vec2f> {(m_p->m_Velocity * 2)};
}