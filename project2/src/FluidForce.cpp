#include "FluidForce.h"

#define IX(i,j) ((i)+(N+2)*(j))

FluidForce::FluidForce(const std::vector<Particle*> &particles) : particles(particles) {
}

void FluidForce::calculateForce(int N, float dt, float * density, float * u_current, float * v_current, float * u_previous, float * v_previous) {

    for(int ii=0; ii< particles.size(); ii++) {
        int x = (int)(particles[ii]->m_Position[0]*N);
        int y = (int)(particles[ii]->m_Position[1]*N);

        int position_index = IX(x, y);
        float density_particle = density[position_index]; 

        if ( density_particle > 0 ) {
            Vec2f current_vel = Vec2f(u_current[position_index], v_current[position_index]);
            Vec2f previous_vel = Vec2f(u_previous[position_index], v_previous[position_index]);
            
            // Calculate the force working on the particle based on velocity in the field 
            Vec2f force = particles[ii]->m_Mass * (current_vel-previous_vel)/dt; 

            particles[ii]->m_Force_acc += force * density_particle;
        }
    }
}

void FluidForce::draw() {
}