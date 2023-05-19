#pragma once

#include "Particle.h"
#include "Constraint.h"
#include <Eigen/Dense>
#include "Eigen/IterativeLinearSolvers"
#include <vector>

class ConstraintSolver {
public:
    ConstraintSolver(std::vector<Particle *> pVector, std::vector<Constraint *> constraintVector);
    void calculateConstraintForce();

private:
    std::vector<Particle*> m_pVector;
    std::vector<Constraint*> m_constraintVector;
};