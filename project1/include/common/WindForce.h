#pragma once
#include "Particle.h"
#include <vector>

class WindForce {

    public:
        WindForce(std::vector<Particle*> pVector);
        virtual ~WindForce(void);
        void calculateWindForce();

    private:
        std::vector<Particle*> const m_pVec;
        float G_const; 
};

