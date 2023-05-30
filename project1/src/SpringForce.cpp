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

void SpringForce::calculateForce()
{
  Vec2f l = m_p1->m_Position - m_p2->m_Position;
  Vec2f i = m_p1->m_Velocity - m_p2->m_Velocity;
  float dist_l = sqrt(l * l);

  if (dist_l == 0.0) {
    return;
  }

  // std::cout << "a: " << a[0] << ", " << a[1] << std::endl;
  // std::cout << "b: " << b[0] << ", " << b[1] << std::endl;
  // std::cout << "l: " << l << std::endl;
  // std::cout << "dist_l: " << dist_l << std::endl;
  // std::cout << "i: " << i << std::endl;
  // Vec2f fp1 = -((m_ks * (dist_l - m_dist) + m_kd * ((i * l) / (dist_l))) * l / dist_l);
  // Vec2f fp1 = -((m_ks * (dist_l - m_dist)));
  // double distance = sqrt(pow(m_p1->m_Position[0] - m_p2->m_Position[0], 2)) + (pow(m_p1->m_Position[1] - m_p2->m_Position[1], 2));
  // std::cout << "Distance: " << distance << std::endl;

  Vec2f fp1 = -(m_ks * (dist_l - m_dist) * ((l) / dist_l));
  Vec2f fp2 = -fp1;
  // std::cout << "fp1: " << fp1[0] << ", " << fp1[1] << std::endl;
  // std::cout << "fp2: " << fp2[0] << ", " << fp2[1] << std::endl;
  // std::cout << "particle 2 force: " << m_p2->m_force[0] << ", " << m_p2->m_force[1] << std::endl;
  m_p1->m_Force_acc += fp1;
  m_p2->m_Force_acc += fp2;
  // std::cout << "particle 2 force: " << m_p2->m_force[0] << ", " << m_p2->m_force[1] << std::endl;

}

std::vector<Particle *> SpringForce::getParticles() {
  return {m_p1, m_p2};
}
