#include <algorithm>
#include "Particle.h"
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include "SpringForce.h"
#include "AngularSpringForce.h"
#include <cmath>

AngularSpringForce::AngularSpringForce(Particle *p1, Particle *p2, Particle *p3, double angle, double ks, double kd): 
    m_p1(p1), m_p2(p2), m_p3(p3), angle(angle), ks(ks), kd(kd){};

void AngularSpringForce::draw() {
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
    glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
    glVertex2f(m_p3->m_Position[0], m_p3->m_Position[1]);
    glEnd();
}

void AngularSpringForce::calculateForce() {
    Vec2f dis12 = m_p1->m_Position - m_p2->m_Position;
    Vec2f dis32 = m_p3->m_Position - m_p2->m_Position;

    // Try to bound the angle between 1 and -1 otherwise I get weird behaviour
    float current_angle = dis12 * dis32 / (norm(dis12) * norm(dis32));
    if (current_angle < -1.0)
        current_angle = 1.0;
    if (current_angle > 1.0)
        current_angle = 1.0;


    float angle_dif = (angle - acos(current_angle)) / 2;

    calculate_force_point(m_p1, -angle_dif);
    calculate_force_point(m_p3, angle_dif);
}

void AngularSpringForce::calculate_force_point(Particle* p, float angle_dif) {
    Vec2f difference = p->m_Position - m_p2->m_Position;
    float x = difference[0] * cos(angle_dif) - difference[1] * sin(angle_dif);
    float y = difference[0] * sin(angle_dif) + difference[1] * cos(angle_dif);
    Vec2f rotation_matrix = Vec2f(x + m_p2->m_Position[0], y + m_p2->m_Position[1]);

    Vec2f difference_dist = rotation_matrix - p->m_Position;
    float distance = norm(difference_dist);

    Vec2f velocity_diff = p->m_Velocity - m_p2->m_Velocity;
    Vec2f force = -((ks * distance) + kd * (velocity_diff * difference_dist / distance)) * distance;

    p->m_Force_acc += force;
    m_p2->m_Force_acc -= force;
}