#include "GravityForce.h"
#if defined(__CYGWIN__) || defined(WIN32)
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <vector>

GravityForce::GravityForce(std::vector<Particle*> pVector) : 
    m_pVec(pVector), G_const(1) {
}

GravityForce::~GravityForce(void) {}

void GravityForce::calculateGravityForce()
{
    for(int i=0; i < m_pVec.size(); i++) {
        m_pVec[i]->m_Force_acc[1] -= m_pVec[i]->m_Mass * G_const;
    }
}