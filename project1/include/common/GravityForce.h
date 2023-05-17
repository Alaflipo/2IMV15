#pragma once
#include "Particle.h"
#include <vector>

class GravityForce {

    public:
        GravityForce(std::vector<Particle*> pVector);
        virtual ~GravityForce(void);
        void update_gravity();

    private:
        std::vector<Particle*> const m_pVec;
        float G_const; 
};

