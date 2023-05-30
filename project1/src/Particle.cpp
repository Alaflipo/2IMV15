#include "Particle.h"
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <vector>

Particle::Particle(const Vec2f & ConstructPos) :
	m_ConstructPos(ConstructPos), m_Position(ConstructPos), 
    m_Velocity(Vec2f(0.0, 0.0)), m_Mass(1.0), m_Force_acc(Vec2f(0.0,0.0))
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
    m_Mass = 1.0;
	m_Force_acc = Vec2f(0.0, 0.0);
}

std::vector<Vec2f> Particle::get_state() {
    return { m_Position, m_Velocity };
}

void Particle::set_state(Vec2f position, Vec2f velocity) {
    m_Position = position;
    m_Velocity = velocity;
}

Vec2f Particle::get_acceleration() { 
    return m_Force_acc / m_Mass;
}

std::vector<Vec2f> Particle::derivEval() {
    return {m_Velocity, get_acceleration()};
}

void Particle::draw()
{
	const double h = 0.03;
	glColor3f(1.f, 1.f, 1.f); 
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();
}

void Particle::clearForce() {
	m_Force_acc = Vec2f(0.0, 0.0);
}
