#pragma once

#include "Particle.h"
#include "gfx/vec2.h"
#include <Eigen/Dense>
#include "Eigen/IterativeLinearSolvers"
#include "Constraint.h"

#include <vector>

using namespace Eigen;

class LineConstraint : public Constraint {
public:
    LineConstraint(Particle* p, const Vec2f & start, const Vec2f & end, const float dist);
    virtual ~LineConstraint(void);

    void draw() override;
    std::vector<Particle*> getParticles() override;

    float getConstraint() override;
    float getConstraintDerivative() override;
    std::vector<Vec2f> getJacobian() override;
    std::vector<Vec2f> getJacobianDerivative() override;


private:

    Particle* const p;
    Vec2f const start;
    Vec2f const end;
    float const dist;

    Vec2f get_closest_point();

};