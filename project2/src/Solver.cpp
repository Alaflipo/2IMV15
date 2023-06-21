#include "Particle.h"

#include <vector>
#include <iostream>
#include <Eigen/Dense>

#define DAMP 0.98f;
void simulation_step( std::vector<Particle*> pVector, float dt )
{
	
	for(int i=0; i<pVector.size(); i++)
	{
        Vec2f accelaration = pVector[i]->get_acceleration();
		pVector[i]->m_Position += dt * pVector[i]->m_Velocity;
		pVector[i]->m_Velocity += dt * accelaration;
        std::cout << pVector[i]->m_Velocity[0] << " " << pVector[i]->m_Velocity[1] << " " << pVector[i]->m_Mass << "\n";
    }
}

using namespace Eigen;

void eulerStep(Particle * p, float dt, std::vector<Vec2f> state, std::vector<Vec2f> derivEval) {
    Vec2f position = state[0] + dt * derivEval[0];
    Vec2f velocity = (state[1] + dt * derivEval[1]) * DAMP;
    p->set_state(position, velocity);
}

void midpointStep(Particle * p, float dt, std::vector<Vec2f> state, std::vector<Vec2f> derivEval) {
    Vec2f position = state[0] + ((dt / 2) * derivEval[0]);
    Vec2f velocity = (state[1] + ((dt / 2) * derivEval[1])) * DAMP;
    p->set_state(position, velocity);
    derivEval = p->derivEval();
    position = state[0] + dt * derivEval[0];
    velocity = (state[1] + dt * derivEval[1]) * DAMP;
    p->set_state(position, velocity);
}

void rungeKuttaStep(Particle * p, float dt, std::vector<Vec2f> state, std::vector<Vec2f> derivEval) {
    Vec2f position = state[0] + ((dt / 2) * derivEval[0]);
    Vec2f velocity = (state[1] + ((dt / 2) * derivEval[1])) * DAMP;
    p->set_state(position, velocity);
    derivEval = p->derivEval();
    position = state[0] + ((dt / 2) * derivEval[0]);
    velocity = (state[1] + ((dt / 2) * derivEval[1])) * DAMP;
    p->set_state(position, velocity);
    derivEval = p->derivEval();
    position = state[0] + dt * derivEval[0];
    velocity = (state[1] + dt * derivEval[1]) * DAMP;
    p->set_state(position, velocity);
}

void simulation_step(std::vector<Particle*> pVector, float dt, int scheme) {	
	for(int i=0; i<pVector.size(); i++)
	{
        Particle * particle = pVector[i];
        std::vector<Vec2f> state = particle->get_state();
        std::vector<Vec2f> derivEval = particle->derivEval();

        if (scheme == 0) {
            eulerStep(particle, dt, state, derivEval);
        } else if (scheme == 1) {
            midpointStep(particle, dt, state, derivEval);
        } else {
            rungeKuttaStep(particle, dt, state, derivEval);
        }
    }
}

