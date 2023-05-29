#include <algorithm>
#include "Particle.h"
#if defined(__CYGWIN__) || defined(WIN32)
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

// void AngularSpringForce::calculateForce() {
//     Vec2f dis12 = m_p1->m_Position - m_p2->m_Position;
//     Vec2f dis32 = m_p3->m_Position - m_p2->m_Position;

//     double angle_point = acos(dis12 * dis32 / (norm(dis12) * norm(dis32)));
//     double difference_angle = (angle - angle_point) / 2;

//     calculate_force(m_p1, -angle_point);
//     calculate_force(m_p3, angle_point);
// }

// void AngularSpringForce::calculate_force(Particle* p, double angle_delta) {
    
//     // calculates difference around a point
//     Vec2f pos_dif = p->m_Position - m_p2->m_Position;
//     double new_pos_x = pos_dif[0] * cos(angle) - pos_dif[1] * sin(angle);
//     double new_pos_y = pos_dif[0] * sin(angle) + pos_dif[1] * cos(angle);
//     Vec2f rotation = Vec2f(new_pos_x + m_p2->m_Position[0], new_pos_y + m_p2->m_Position[1]);

//     Vec2f distance_shift = rotation - p->m_Position;
//     double distance_shift_norm = norm(distance_shift);

//     Vec2f velocity_diff = p->m_Velocity - m_p2->m_Velocity;
//     Vec2f force = -((ks * distance_shift_norm) + kd * (velocity_diff * distance_shift / distance_shift_norm)) * distance_shift_norm;

//     apply_force(p, force);
// }

// double AngularSpringForce::clip(double value, double lower, double upper) {
//     return std::max(lower, std::min(value, upper));
// }

// void AngularSpringForce::apply_force(Particle* p, Vec2f force) {
//     p->m_Force_acc += force;
//     m_p2->m_Force_acc -= force;
// }


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

// Vec2f pos12 = m_p1->m_Position - m_p2->m_Position;
	// Vec2f pos32 = m_p3->m_Position - m_p2->m_Position;
	// Vec2f vel12 = m_p1->m_Velocity - m_p2->m_Velocity;
	// Vec2f vel32 = m_p3->m_Velocity - m_p2->m_Velocity;

	// float dis12 = norm(m_p1->m_Position - m_p2->m_Position);
    // float dis12_squared = pow(dis12, 2);
	// float dis32 = norm(m_p3->m_Position - m_p2->m_Position);
    // float dis32_squared = pow(dis32, 2);

	// float dis_dot = (pos12[0] * pos32[0] + pos12[1] * pos32[1]);
    // float D = (pos12[0] * pos32[1] - pos12[1] * pos32[0]);
    // float cosine_angle = 2 * (pow(dis_dot, 2) / dis12_squared / dis32_squared) - 1;

	// float alpha = 0;
	// if (cosine_angle <= -1) 
	// 	alpha = M_PI;
	// else if (cosine_angle >= 1) 
	// 	alpha = 0;
	// else 
	// 	alpha = acosf(cosine_angle);

	// float angle = (alpha/2) * 180. / M_PI;
	// if (dis_dot < 0)
	// 	angle = 180 - angle;
	// if (D < 0)
	// 	angle = -angle;
	
	// // We scale the angle otherwise it becomes to big
	// angle = 0.005f*angle;

	// m_p1->m_Force_acc[0] += ((ks * angle) / dis12 * pos12[1]) + (kd * vel12[0]);
	// m_p1->m_Force_acc[1] += ((ks * angle) / dis12 * pos12[0]) + (kd * vel12[1]);
	// m_p3->m_Force_acc[0] -= ((ks * angle) / dis32 * pos32[1]) + (kd * vel32[0]);
	// m_p3->m_Force_acc[1] -= ((ks * angle) / dis32 * pos32[0]) + (kd * vel32[1]);
