#include "FixedObject.h"
#include <vector>
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <Eigen/Dense>
#include <gfx/vec2.h>

FixedObject::FixedObject(std::vector<Vec2f> initPoints) {
    points = initPoints;
}

FixedObject::~FixedObject(void) {}

std::vector<Vec2f> FixedObject::getPoints() {
    return points;
}

void FixedObject::drawObject() {
    glColor3f(1, 0, 0);
    glBegin(GL_POLYGON);
    for (int i = 0; i < points.size(); i ++) {
        glVertex2f(points[i][0], points[i][1]);
    }
    glEnd();
}