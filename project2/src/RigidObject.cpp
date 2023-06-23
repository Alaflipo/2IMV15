#include "RigidObject.h"
#include <vector>
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <Eigen/Dense>
#include <gfx/vec2.h>

static Vector2f VecEigen(Vec2f vec) {
    return Vector2f(vec[0], vec[1]);
}

static Vec2f EigenVec(Vector2f vec) {
    return Vec2f(vec[0], vec[1]);
}

RigidObject::RigidObject(std::vector<Particle *> initParticles, int n) {
    particles = initParticles;
    N = n;

    x0 = Vector2f(0,0);
    M = 0.0;
    for (Particle * particle : particles) {
        Vec2f pos = particle->m_Mass * particle->m_Position;
        x0 += Vector2f(pos[0], pos[1]);
        M += particle->m_Mass;
    }
    x0 = x0 / M;
    for (Particle* particle : particles) {
        particle->m_ConstructPos = particle->m_Position - Vec2f(x0[0], x0[1]);
        particle->m_Position = particle->m_ConstructPos;
    }

    reset();

    IBody = Matrix2f::Identity();
    for (Particle * particle: particles) {
        Vector2f pos = VecEigen(particle->m_Position);
        IBody += pos.transpose() * pos  * Matrix2f::Identity() - pos * pos.transpose();
    }
    IBodyInv = IBody.inverse();
    IInv = R * IBodyInv * R.transpose();
    omega = IInv.norm() * angularMomentum;
}

RigidObject::~RigidObject(void) {}

std::vector<Vec2f> RigidObject::getPoints() {
    std::vector<Vec2f> positions;
    for (Particle * particle: particles) {
        Vec2f particlePos = particle->get_state()[0];
        positions.push_back(Vec2f(x[0] + particlePos[0], x[1] + particlePos[1]));
    }
    return positions;
}

void RigidObject::setState(VectorXf state) {
    x[0] = state[0]; x[1] = state[1];
    R(0, 0) = state[2]; R(0, 1) = state[3]; R(1, 0) = state[4]; R(1, 1) = state[5];
    linearMomentum[0] = state[6]; linearMomentum[1] = state[7];
    angularMomentum = state[8];

    for (Particle * particle: particles) {
        Vector2f newPos = R * VecEigen(particle->m_Position);
        particle->m_Position = EigenVec(newPos);
    }

    v = linearMomentum / M;
    IInv = R * IBody.inverse() * R.transpose();
    omega = IInv.norm() * angularMomentum;
}

VectorXf RigidObject::getState() {
    VectorXf state(9);
    state[0] = x[0]; state[1] = x[1];
    state[2] = R(0, 0); state[3] = R(0, 1); state[4] = R(1, 0); state[5] = R(1, 1);
    state[6] = linearMomentum[0]; state[7] = linearMomentum[1];
    state[8] = angularMomentum;
    return state;
}

VectorXf RigidObject::derivEval() {
    F = Vector2f(0, 0);
    tau = 0;
    for (Particle * particle: particles) {
        Vector2f force = VecEigen(particle->m_Force_acc);
        F += force;

        Vector2f pos = VecEigen(particle->m_Position);
        tau += pos[0] * force[1] - pos[1] * force[0];
    }

    Matrix2f Rdot = Matrix2f::Zero();
    Matrix2f r = Matrix2f::Zero();
    float rotScale = 0.2;
    r(0, 1) = -std::sin(omega * rotScale);
    r(1, 0) = std::sin(omega * rotScale);
    Rdot = R * r;

    VectorXf state(9);
    state[0] = v[0]; state[1] = v[1];
    state[2] = Rdot(0, 0); state[3] = Rdot(0, 1); state[4] = Rdot(1, 0); state[5] = Rdot(1, 1);
    state[6] = F[0]; state[7] = F[1];
    state[8] = tau;
    return state;
}

void RigidObject::clearForce() {
    F = Vector2f(0, 0);
    for (Particle * particle: particles) {
        particle->clearForce();
    }
}

void RigidObject::reset() {
    x = x0;
    R = Matrix2f::Identity();
    linearMomentum = Vector2f(0, 0);
    angularMomentum = 0;

    v = Vector2f(0, 0);
    F = Vector2f(0, 0);
    tau = 0;

    IInv = IBodyInv;
    omega = IInv.norm() * angularMomentum;

    for (Particle * particle: particles) {
        particle->reset();
    }
}

void RigidObject::drawObject() {
    std::vector<Vec2f> points = getPoints();
    glColor3f(0, 1, 0);
    glBegin(GL_POLYGON);
    for (int i = 0; i < points.size(); i ++) {
        glVertex2f(points[i][0], points[i][1]);
    }
    glEnd();
}

Vector2f RigidObject::VecEigen(Vec2f vec) {
    return Vector2f(vec[0], vec[1]);
}

Vec2f RigidObject::EigenVec(Vector2f vec) {
    return Vec2f(vec[0], vec[1]);
}