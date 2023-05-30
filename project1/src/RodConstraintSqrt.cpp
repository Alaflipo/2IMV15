#include "RodConstraintSqrt.h"
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
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


RodConstraintSqrt::RodConstraintSqrt(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

RodConstraintSqrt::~RodConstraintSqrt(void) {}

void RodConstraintSqrt::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0, 0);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0, 0); 
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

std::vector<Particle *> RodConstraintSqrt::getParticles()
{
  std::vector<Particle *> particles;
  particles.push_back(m_p1);
  particles.push_back(m_p2);
  return particles;
}

float RodConstraintSqrt::getConstraint()
{
    Vec2f pos_diff = m_p1->m_Position - m_p2->m_Position;
    return sqrt(pow(pos_diff[0], 2) + pow(pos_diff[1], 2)) - m_dist;
}

float RodConstraintSqrt::getConstraintDerivative()
{
    Vec2f pos_diff = m_p1->m_Position - m_p2->m_Position;
    Vec2f vel_diff = m_p1->m_Velocity - m_p2->m_Velocity;
    return (pos_diff * vel_diff) / (pos_diff * pos_diff);
}

std::vector<Vec2f> RodConstraintSqrt::getJacobian()
{
    Vec2f pos_diff1 = m_p1->m_Position - m_p2->m_Position;
    Vec2f pos_diff2 = m_p2->m_Position - m_p1->m_Position;
    Vec2f j_p1 = pos_diff1 * 2;
    Vec2f j_p2 = pos_diff2 * 2;
    std::vector<Vec2f> res;
    res.push_back(j_p1);
    res.push_back(j_p2);
    return res;
}

std::vector<Vec2f> RodConstraintSqrt::getJacobianDerivative()
{
    Vec2f j_deriv1 = (m_p1->m_Velocity - m_p2->m_Velocity) * 2;
    Vec2f j_deriv2 = (m_p2->m_Velocity - m_p1->m_Velocity) * 2;
    std::vector<Vec2f> res;
    res.push_back(j_deriv1);
    res.push_back(j_deriv2);
    return res;
}
