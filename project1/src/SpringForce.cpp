#include "SpringForce.h"
#include <GL/glut.h>
#include "cmath"

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

void SpringForce::calculateForce(bool implicitEuler)
{
  Vec2f l = m_p1->m_Position - m_p2->m_Position;
  Vec2f i = m_p1->m_Velocity - m_p2->m_Velocity;
  float dist_l = sqrt(l * l);

  if (dist_l == 0.0) {
    return;
  }

  Vec2f fp1;
  Vec2f fd1; Vec2f fd2;
  if (implicitEuler) {
    fp1 = -(m_ks * (dist_l - m_dist) * ((l) / dist_l));

    fd1 = -m_kd * i;
    fd2 = -fd1;
    m_p1->m_Force_acc += fd1;
    m_p2->m_Force_acc += fd2;
  } else {
    fp1 = -((m_ks * (dist_l - m_dist) + m_kd * ((i * l) / (dist_l))) * l / dist_l);
  }
  Vec2f fp2 = -fp1;

  m_p1->m_Force_acc += fp1;
  m_p2->m_Force_acc += fp2;
}

std::vector<Particle *> SpringForce::getParticles() {
  return {m_p1, m_p2};
}
