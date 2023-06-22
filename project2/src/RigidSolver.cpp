#include "RigidObject.h"

#include <vector>
#include <Eigen/Dense>

void simulationStep(RigidObject * rb, float dt) {
    VectorXf state = rb->getState();
    VectorXf deriv = rb->derivEval();

    VectorXf newState = state + dt * deriv;
    rb->setState(newState);

    deriv = rb->derivEval();
    newState[0] = state[0] + dt * deriv[0];
    newState[1] = state[1] + dt * deriv[1];
    newState[2] = state[2] + dt * deriv[2]; 
    newState[3] = state[3] + dt * deriv[3];
    newState[4] = state[4] + dt * deriv[4];
    newState[5] = state[5] + dt * deriv[5];
    rb->setState(newState);
}

void rigidSimulationStep(std::vector<RigidObject*> rigidObjects, float dt) {
    for (int i = 0; i < rigidObjects.size(); i++) {
        simulationStep(rigidObjects[i], dt);
    }
}

