
#include "Particle.h"
#include <vector>
#include <gfx/vec2.h>

class FluidForce {
public:
    FluidForce(const std::vector<Particle *> particles);
    void calculateForce(int N, float dt, float * density, float * u_current, float * v_current, float * u_previous, float * v_previous);
    void draw();

    std::vector<Particle*> particles; 
};