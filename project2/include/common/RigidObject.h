#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <gfx/vec2.h>
#include "Object.h"
#include "Particle.h"

using namespace Eigen;

class RigidObject : public Object {
    public:
        RigidObject(std::vector<Particle *> initParticles, int n);
        virtual ~RigidObject(void);
        std::vector<Vec2f> getPoints() override;
        void setState(VectorXf state);
        VectorXf getState();
        VectorXf derivEval();

        void clearForce();
        void reset();
        void drawObject() override;
    private:
        int N;
        double M;
        std::vector<Particle *> particles;

        // Current state
        Vector2f x;
        Matrix2f R;
        Vector2f linearMomentum;
        float angularMomentum;

        // State derivative
        VectorXf v;
        MatrixXf Romgea;
        float force;
        float torque;

        // Helper
        Matrix2f I;
};


