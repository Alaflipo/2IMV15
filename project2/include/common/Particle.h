#pragma once

#include <gfx/vec2.h>
#include <vector>

class Particle
{
public:

	Particle(const Vec2f & ConstructPos);
	virtual ~Particle(void);
	void reset();

    std::vector<Vec2f> get_state();
    void set_state(Vec2f position, Vec2f velocity); 
    Vec2f get_acceleration();
	std::vector<Vec2f> derivEval();

	void draw();
	void clearForce();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
    float m_Mass;
	Vec2f m_Force_acc;
};
