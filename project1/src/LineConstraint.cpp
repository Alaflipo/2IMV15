#include "LineConstraint.h"
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif


LineConstraint::LineConstraint(Particle* p, const Vec2f & start, const Vec2f & end, float dist) :
        p(p), start(start), end(end), dist(dist) {
}

LineConstraint::~LineConstraint(void) {}

std::vector<Particle*> LineConstraint::getParticles() {
    return std::vector<Particle*> {p};
}

void LineConstraint::draw()
{
    // // glBegin(GL_LINES);
    // glColor3f(1.0, 0.0, 0.0);
    // glVertex2f(start[0], start[1]);
    // glVertex2f(end[0], end[1]);
    // glEnd();

    // Vec2f vertex = get_closest_point();
    // glBegin(GL_LINES);
    // glColor3f(1.0, 0.0, 0.0);
    // glVertex2f(vertex[0], vertex[1]);
    // glVertex2f(p->m_Position[0], p->m_Position[1]);
    // glEnd();
}

Vec2f LineConstraint::get_closest_point() {
    float diff_x = (start[0] - end[0]);
    float diff_y = (start[1] - end[1]);
    float slope =  diff_y / diff_x;
    float intersection = start[1] - slope * start[0];

    return Vec2f(p->m_Position[0], slope*p->m_Position[0]+intersection);
}

float LineConstraint::getConstraint() {
    Vec2f difference = p->m_Position - get_closest_point();
    return difference[0] * difference[0] + difference[1] * difference[1] - dist * dist;
}

float LineConstraint::getConstraintDerivative() {
    return 2 * (p->m_Position - get_closest_point()) * 2 * p->m_Velocity;
}

std::vector<Vec2f> LineConstraint::getJacobian() {
    return std::vector<Vec2f> {p->m_Position - get_closest_point() * 2};
}

std::vector<Vec2f> LineConstraint::getJacobianDerivative() {
    return std::vector<Vec2f> {p->m_Velocity * 2};
}