#include <ConstraintSolver.h>

using namespace Eigen;

ConstraintSolver::ConstraintSolver(std::vector<Particle*> pVector, std::vector<Constraint*> constraintVector ):
        m_pVector(pVector), m_constraintVector(constraintVector)
{
}

void ConstraintSolver::calculateConstraintForce()
{
    const int dimensions = 2;
    const float ks = 2;
    const float kd = 2;

    const int pVec_size = m_pVector.size();
    const int num_constraints = m_constraintVector.size();

    VectorXf q_dot = VectorXf::Zero(pVec_size * dimensions);
    VectorXf Q = VectorXf::Zero(pVec_size * dimensions);
    MatrixXf W = MatrixXf::Zero(pVec_size * dimensions, pVec_size * dimensions);
    MatrixXf C = VectorXf::Zero(num_constraints);
    MatrixXf C_dot = VectorXf::Zero(num_constraints);
    MatrixXf J = MatrixXf::Zero(num_constraints, pVec_size * dimensions);
    MatrixXf Jt = MatrixXf::Zero(pVec_size * dimensions, num_constraints);
    MatrixXf J_dot = MatrixXf::Zero(num_constraints, pVec_size * dimensions);

    for (int i = 0; i < m_pVector.size(); i++) {
        for (int j = 0; j < dimensions; j++) {
            q_dot[i * dimensions + j] = m_pVector[i]->m_Velocity[j];
            Q[i * dimensions + j] = m_pVector[i]->m_Force_acc[j];
            W(i * dimensions + j, i * dimensions + j) = 1 / m_pVector[i]->m_Mass;
        }
    }

    for ( int c = 0; c < m_constraintVector.size(); c++ ) {
        Constraint* constraint = m_constraintVector[c];

        C(c) = constraint->getConstraint();
        C_dot(c) = constraint->getConstraintDerivative();
        std::vector<Vec2f> jacobian = constraint->getJacobian();
        std::vector<Vec2f> jacobianDerivative = constraint->getJacobianDerivative();
        std::vector<Particle*> particles = constraint->getParticles();
        for (int i = 0; i < particles.size(); i++) {
            std::vector<Particle *>::iterator iterable = std::find(m_pVector.begin(), m_pVector.end(), particles[i]);
            int affectedParticleIndex = distance(m_pVector.begin(), iterable);
            if ( affectedParticleIndex < m_pVector.size() ) {
                for (int j = 0; j < dimensions; j++) {
                    int index = affectedParticleIndex * dimensions + j;
                    J(c, index) = jacobian[i][j];
                    Jt(index, c) = jacobian[i][j];
                    J_dot(c, index ) = jacobianDerivative[i][j];
                }
            }
        }
    }

    VectorXf JWJt_lambda = (-1 * J_dot * q_dot) - ((J * W) * Q) - (ks * C) - (kd * C_dot);
    ConjugateGradient<MatrixXf, Lower|Upper> cg;
    cg.compute((J * W) * Jt);
    Eigen::Solve<Eigen::ConjugateGradient<Eigen::MatrixXf, 3>, Eigen::VectorXf> lambda = cg.solve(JWJt_lambda);
    VectorXf Q_hat = Jt * lambda;

    for ( int p = 0; p < m_pVector.size(); p++ ) {
        m_pVector[p]->m_Force_acc += Vec2f(Q_hat[p * dimensions], Q_hat[(p * dimensions) + 1]);
    }
}