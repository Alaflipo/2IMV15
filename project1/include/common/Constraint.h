#pragma once

#include <vector>
#include "gfx/vec2.h"
#include "Particle.h"

class Constraint {
public:
    virtual void draw() { return; };
    virtual std::vector<Particle *> getParticles() { return std::vector<Particle *> (); };
    virtual float getConstraint() { return 0; };
    virtual float getConstraintDerivative() { return 0; };
    virtual std::vector<Vec2f> getJacobian() { return std::vector<Vec2f> (); };
    virtual std::vector<Vec2f> getJacobianDerivative() { return std::vector<Vec2f> (); };
};