#include "Wall.h"
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <vector>

Wall::Wall(Vec2f p1, Vec2f p2, float eps) : 
    m_p1(p1), m_p2(p2), m_eps(eps) {
}

Wall::~Wall(void) {}

void Wall::detectCollisions(std::vector<Particle *> particles)
{
    for (Particle * particle: particles) {
        Vec2f normal = Vec2f(0, -particle->m_Force_acc[1]);       

        if (norm(particle->m_Position - Vec2f(particle->m_Position[0], m_p1[1])) < m_eps && normal * particle->m_Force_acc < 0) {
            handleCollision(particle, normal);
        }
    }
}

void Wall::handleCollision(Particle * particle, Vec2f normal) 
{
    	Vec2f tan = particle->m_Force_acc + normal;
        if (norm(particle->m_Position - Vec2f(particle->m_Position[0], m_p1[1])) < 0.01) {
            particle->m_Force_acc[1] = 0;
        } else {
            float divisor = particle->m_Force_acc[1] / normal[1];
            particle->m_Force_acc += divisor * 2.5 * normal;
        }
}

void Wall::drawWall()
{
    glBegin( GL_LINES );
    glColor3f(0.0, 0.7, 0.0);
    glVertex2f( m_p1[0], m_p1[1] );
    glColor3f(0.0, 0.7, 0.0);
    glVertex2f( m_p2[0], m_p2[1] );
    glEnd();
}