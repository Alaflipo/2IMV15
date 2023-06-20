#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>
#include "Object.h"

class FixedObject : public Object {
    public:
        FixedObject(std::vector<Vec2f> initPoints);
        virtual ~FixedObject(void);
        std::vector<Vec2f> getPoints() override;
        void drawObject() override;
    private:
        std::vector<Vec2f> points;
};


