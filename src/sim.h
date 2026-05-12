#pragma once

#include "mesh.h"

#include <btBulletDynamicsCommon.h>

#include <vector>

// Simulation hyper-parameters (tweak to taste / match the paper's setup)
struct simParams {
    int    numTrials          = 5000;    // number of random drops
    double dropHeight         = 5.0;
    double gravity            = -1.0;
    double friction           = 0.8;    // high friction → energy dissipates fast
    double restitution        = 0.2;    // low bounce
    double linearDamping      = 0.3;
    double angularDamping     = 0.3;
    double fixedTimeStep      = 1.0 / 60.0;
    int    maxSubSteps        = 10;
    double sleepLinThreshold  = 0.05;   // body is "asleep" when speed < this
    double sleepAngThreshold  = 0.05;
    int    maxStepsPerTrial   = 6000;   // safety cap (~25 s at 240 Hz)
    double faceAlignThreshold = 0.92;   // dot(face_normal, -Y) > this → landed
};

// ─────────────────────────────────────────────────────────────────────────────
// BulletSimulation
//
// Wraps a Bullet3 world, drops the mesh N times from a random orientation,
// waits for it to sleep, then asks Mesh::identifyRestingFace() which hull
// face is touching the ground.  Results are stored as FaceData<FaceResult>
// on the Mesh and can be visualised via Mesh::showFaceProbabilities().
// ─────────────────────────────────────────────────────────────────────────────
class BulletSimulation {
public:

    using Params = simParams;

    explicit BulletSimulation(Params p = {}) : _p(p) {}

    // Run all trials, populate results on mesh, return face probabilities
    // indexed by geometry-central Face index.
    std::vector<FaceResult> runTrials(Mesh& mesh);

private:
    Params _p;

    // Allocate / free a fresh world each trial so state never bleeds over
    struct World {
        btDefaultCollisionConfiguration*     colConfig   = nullptr;
        btCollisionDispatcher*               dispatcher  = nullptr;
        btDbvtBroadphase*                    broadphase  = nullptr;
        btSequentialImpulseConstraintSolver* solver      = nullptr;
        btDiscreteDynamicsWorld*             world       = nullptr;
        btRigidBody*                         ground      = nullptr;
        btRigidBody*                         body        = nullptr;
        btConvexHullShape*                   shape       = nullptr;
        // motion states & shapes owned here so they get cleaned up
        btDefaultMotionState*                groundMS    = nullptr;
        btDefaultMotionState*                bodyMS      = nullptr;
        btCollisionShape*                    groundShape = nullptr;
    };

    World  buildWorld(Mesh& mesh, const btQuaternion& rotation);
    void   destroyWorld(World& w);
    bool   bodyIsAsleep(btRigidBody* body) const;

    // Given the body's final transform, find which hull face is face-down
    // (i.e. its outward normal is most aligned with -Y in world space).
    int    identifyRestingFace(Mesh& mesh, const btTransform& finalTransform) const;
};
