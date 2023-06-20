#include "RigidObject.h"
#include <vector>
#include <GL/glut.h>
#include <Eigen/Dense>
#include <gfx/vec2.h>

RigidObject::RigidObject(std::vector<Vec2f> initPoints) {
    points = initPoints;
}

RigidObject::~RigidObject(void) {}

std::vector<Vec2f> RigidObject::getPoints() {
    return points;
}

void RigidObject::drawObject() {
    glColor3f(1, 0, 0);
    glBegin(GL_POLYGON);
    for (int i = 0; i < points.size(); i ++) {
        glVertex2f(points[i][0], points[i][1]);
    }
    glEnd();
}