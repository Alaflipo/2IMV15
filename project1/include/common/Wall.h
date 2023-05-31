#pragma once
#include "Particle.h"
#include <vector>

class Wall {

    public:
        Wall(Vec2f p1, Vec2f p2, float eps);
        virtual ~Wall(void);
        void detectCollisions(std::vector<Particle *> particles);
        void drawWall();

    private:
        void handleCollision(Particle * particle, Vec2f normal);
        Vec2f m_p1;
        Vec2f m_p2;
        float m_eps;
};