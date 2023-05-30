#pragma once

#include "Particle.h"
#include "gfx/vec2.h"
#include <Eigen/Dense>
#include "Eigen/IterativeLinearSolvers"
#include "Constraint.h"

#include <vector>

using namespace Eigen;

class RodConstraintSqrt : public Constraint {
 public:
  RodConstraintSqrt(Particle *p1, Particle * p2, double dist);
  virtual ~RodConstraintSqrt(void);

  void draw() override;
  std::vector<Particle*> getParticles() override;
  float getConstraint() override;
  float getConstraintDerivative() override;
  std::vector<Vec2f> getJacobian() override;
  std::vector<Vec2f> getJacobianDerivative() override;

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};