#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>
#include "Object.h"

class RigidObject : public Object {
    public:
        RigidObject(std::vector<Vec2f> initPoints);
        virtual ~RigidObject(void);
        std::vector<Vec2f> getPoints() override;
        void drawObject() override;
    private:
        std::vector<Vec2f> points;
};


