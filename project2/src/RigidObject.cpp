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
    x[0] = state[0];
    x[1] = state[1];
    R(0, 0) = state[2];
    R(0, 1) = state[3];
    R(1, 0) = state[4];
    R(1, 1) = state[5];
    linearMomentum[0] = state[6];
    linearMomentum[1] = state[7];
    angularMomentum = state[8];

    if (v.norm() <= 0.0001) {
        x = Vector2f((float)((int)(x[0] * N) / N), (float)((int)(x[1] * N) / N));
    }

    for (Particle * particle: particles) {
        Vector2f newPos = R * VecEigen(particle->m_Position);
        particle->m_Position = EigenVec(newPos);
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