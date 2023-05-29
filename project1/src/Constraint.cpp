#include "Constraint.h"

void draw() {};
std::vector<Particle*> getParticles() { return std::vector<Particle*>({}); };
float getConstraint() { return 0; };
float getConstraintDerivative() { return 0; };
std::vector<Vec2f> getJacobian() { return std::vector<Vec2f>((0.0)); };
std::vector<Vec2f> getJacobianDerivative() { return std::vector<Vec2f>((0.0)); };