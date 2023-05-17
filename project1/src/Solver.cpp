#include "Particle.h"

#include <vector>
#include <iostream>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)
void simulation_step( std::vector<Particle*> pVector, float dt )
{
	
	for(int i=0; i<pVector.size(); i++)
	{
        Vec2f accelaration = pVector[i]->get_acceleration();
		pVector[i]->m_Position += dt * pVector[i]->m_Velocity;
		pVector[i]->m_Velocity += dt * accelaration;
        pVector[i]->m_Force_acc = 0.0;
        std::cout << pVector[i]->m_Velocity[0] << " " << pVector[i]->m_Velocity[1] << " " << pVector[i]->m_Mass << "\n";
    }

}

