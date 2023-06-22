#include "RigidObject.h"

#include <vector>
#include <Eigen/Dense>

void compute_rigidObject(RigidObject* rb, float dt) {
    VectorXf old = rb->getState();
    VectorXf deriv = rb->derivEval();

    VectorXf newState = old + dt * deriv;
    rb->setState(newState);

    VectorXf new_deriv = rb->derivEval();
    newState[0] = old[0] + dt * new_deriv[0];
    newState[1] = old[1] + dt * new_deriv[1];
    newState[2] = old[2] + dt * new_deriv[2]; 
    newState[3] = old[3] + dt * new_deriv[3];
    newState[4] = old[4] + dt * new_deriv[4];
    newState[5] = old[5] + dt * new_deriv[5];
    rb->setState(newState);
}

void rigidSimulationStep(std::vector<RigidObject*> rigidObjects, float dt) {
    for (int i = 0; i < rigidObjects.size(); i++) {
        compute_rigidObject(rigidObjects[i], dt);
    }
}

