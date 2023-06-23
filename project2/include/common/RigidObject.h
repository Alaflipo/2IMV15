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

        std::vector<Particle *> particles;
        Vector2f x;
        VectorXf v;
    private:
        int N;
        double M;
        Vector2f x0;

        // Current state
        Matrix2f R;
        Vector2f linearMomentum;
        float angularMomentum;

        // State derivative
        MatrixXf Romgea;
        VectorXf F;
        float tau;

        // Helper
        Matrix2f IBody;
        Matrix2f IInv;
        Matrix2f IBodyInv;
        float omega;

        Vector2f VecEigen(Vec2f vec);
        Vec2f EigenVec(Vector2f vec);
};


