#pragma once

#include "Particle.h"
#include "gfx/vec2.h"
#include "Constraint.h"
#include "SpringForce.h"
#include <map>

class AngularSpringForce {

public:
    AngularSpringForce(Particle *p1, Particle * p2, Particle * p3, double angle, double ks, double kd);
    void draw();
    void calculateForce();

private:
    Particle * const m_p1;  // particle 1
    Particle * const m_p2;  // particle 2 
    Particle * const m_p3;  // particle 3
    double const angle;      // constrained angle   
    double const ks;        // spring strength
    double const kd;        // spring strength

    void calculate_force_point(Particle* p, float angle_dif);
};