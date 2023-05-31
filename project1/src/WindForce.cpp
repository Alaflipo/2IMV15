#include "WindForce.h"
#if defined(__CYGWIN__) || defined(WIN32) || __linux__
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
#include <vector>

WindForce::WindForce(std::vector<Particle*> pVector) : 
    m_pVec(pVector), G_const(0.7) {
}

WindForce::~WindForce(void) {}

void WindForce::calculateWindForce()
{
    for(int i=0; i < m_pVec.size(); i++) {
        m_pVec[i]->m_Force_acc[0] += m_pVec[i]->m_Mass * G_const;
    }
}