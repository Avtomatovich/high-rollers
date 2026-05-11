#include "sim.h"

#include <Eigen/Geometry>

#include <iostream>
#include <random>
#include <chrono>
#include <limits>

static btQuaternion randomUnitQuaternion(std::mt19937& rng)
{
    // Shoemake's uniform random rotation on SO(3)
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    double u1 = uniform(rng);
    double u2 = uniform(rng) * 2.0 * M_PI;
    double u3 = uniform(rng) * 2.0 * M_PI;

    double s1 = std::sqrt(1.0 - u1);
    double s2 = std::sqrt(u1);

    btQuaternion q(
        static_cast<btScalar>(s1 * std::sin(u2)),
        static_cast<btScalar>(s1 * std::cos(u2)),
        static_cast<btScalar>(s2 * std::sin(u3)),
        static_cast<btScalar>(s2 * std::cos(u3))
        );
    q.normalize();
    return q;
}

BulletSimulation::World BulletSimulation::buildWorld(Mesh& mesh,
                                                     const btQuaternion& rotation)
{
    World w;

    // ── Dynamics world ────────────────────────────────────────────────────────
    w.colConfig  = new btDefaultCollisionConfiguration();
    w.dispatcher = new btCollisionDispatcher(w.colConfig);
    w.broadphase = new btDbvtBroadphase();
    w.solver     = new btSequentialImpulseConstraintSolver();
    w.world      = new btDiscreteDynamicsWorld(
        w.dispatcher, w.broadphase, w.solver, w.colConfig);
    w.world->setGravity(btVector3(0, static_cast<btScalar>(_p.gravity), 0));

    // ── Ground plane ──────────────────────────────────────────────────────────
    w.groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    btTransform groundTransform;
    groundTransform.setIdentity();
    w.groundMS = new btDefaultMotionState(groundTransform);

    btRigidBody::btRigidBodyConstructionInfo groundCI(
        0, w.groundMS, w.groundShape, btVector3(0, 0, 0));
    groundCI.m_friction    = static_cast<btScalar>(_p.friction);
    groundCI.m_restitution = static_cast<btScalar>(_p.restitution);
    w.ground = new btRigidBody(groundCI);
    w.world->addRigidBody(w.ground);

    // ── Convex hull shape (vertices shifted so COM is at origin) ──────────────
    w.shape = new btConvexHullShape();

    for (const Vertex& v : mesh.getHull().vertices()) {
        Vector3 pos = mesh.getHullGeom().vertexPositions[v];
        w.shape->addPoint(btVector3(
            static_cast<btScalar>(pos.x - mesh.getCenterOfMass().x),
            static_cast<btScalar>(pos.y - mesh.getCenterOfMass().y),
            static_cast<btScalar>(pos.z - mesh.getCenterOfMass().z)
            ));
    }
    w.shape->recalcLocalAabb();

    // ── Rigid body ─────────────────────────────────────────────────────────────
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setRotation(rotation);
    startTransform.setOrigin(btVector3(0, static_cast<btScalar>(_p.dropHeight), 0));

    btScalar mass = 1.f;
    btVector3 inertia(0, 0, 0);
    w.shape->calculateLocalInertia(mass, inertia);

    w.bodyMS = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo bodyCI(mass, w.bodyMS, w.shape, inertia);
    bodyCI.m_friction             = static_cast<btScalar>(_p.friction);
    bodyCI.m_restitution          = static_cast<btScalar>(_p.restitution);
    bodyCI.m_linearDamping        = static_cast<btScalar>(_p.linearDamping);
    bodyCI.m_angularDamping       = static_cast<btScalar>(_p.angularDamping);

    // Set Bullet's deactivation (sleep) thresholds
    bodyCI.m_linearSleepingThreshold  = static_cast<btScalar>(_p.sleepLinThreshold);
    bodyCI.m_angularSleepingThreshold = static_cast<btScalar>(_p.sleepAngThreshold);

    w.body = new btRigidBody(bodyCI);
    w.body->setActivationState(ACTIVE_TAG);
    w.world->addRigidBody(w.body);

    return w;
}

// ─────────────────────────────────────────────────────────────────────────────
// BulletSimulation::destroyWorld
// ─────────────────────────────────────────────────────────────────────────────
void BulletSimulation::destroyWorld(World& w)
{
    if (w.world)  { w.world->removeRigidBody(w.body);  w.world->removeRigidBody(w.ground); }
    delete w.body;        w.body        = nullptr;
    delete w.bodyMS;      w.bodyMS      = nullptr;
    delete w.shape;       w.shape       = nullptr;
    delete w.ground;      w.ground      = nullptr;
    delete w.groundMS;    w.groundMS    = nullptr;
    delete w.groundShape; w.groundShape = nullptr;
    delete w.world;       w.world       = nullptr;
    delete w.solver;      w.solver      = nullptr;
    delete w.broadphase;  w.broadphase  = nullptr;
    delete w.dispatcher;  w.dispatcher  = nullptr;
    delete w.colConfig;   w.colConfig   = nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// BulletSimulation::bodyIsAsleep
// ─────────────────────────────────────────────────────────────────────────────
bool BulletSimulation::bodyIsAsleep(btRigidBody* body) const
{
    if (body->getActivationState() == ISLAND_SLEEPING ||
        body->getActivationState() == WANTS_DEACTIVATION)
        return true;

    // Also check velocities manually — Bullet sometimes delays the flag
    btScalar linSpd = body->getLinearVelocity().length();
    btScalar angSpd = body->getAngularVelocity().length();
    return (linSpd < static_cast<btScalar>(_p.sleepLinThreshold) &&
            angSpd < static_cast<btScalar>(_p.sleepAngThreshold));
}

// ─────────────────────────────────────────────────────────────────────────────
// BulletSimulation::identifyRestingFace
//
// Strategy: transform each hull face-normal into world space, find the one
// whose world-space normal is most closely aligned with -Y (pointing down
// into the ground).  That face is the one resting on the ground.
// ─────────────────────────────────────────────────────────────────────────────
int BulletSimulation::identifyRestingFace(Mesh& mesh,
                                          const btTransform& finalTransform) const
{
    btMatrix3x3 btRot = finalTransform.getBasis();
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R(r, c) = static_cast<double>(btRot[r][c]);

    Eigen::Vector3d downWorld(0, -1, 0);

    double bestDot     = -std::numeric_limits<double>::infinity();
    int    bestFaceIdx = 0;

    for (const Face& f : mesh.getHull().faces()) {
        const Vector3& fn = mesh.getHullGeom().faceNormals[f];
        Eigen::Vector3d worldNormal = R * Eigen::Vector3d(fn.x, fn.y, fn.z);
        double d = worldNormal.dot(downWorld);
        if (d > bestDot) {
            bestDot     = d;
            bestFaceIdx = static_cast<int>(f.getIndex());
        }
    }

    // If Bullet landed on a non-stable face (stuck on edge/vertex numerically),
    // remap to the nearest stable face by comparing local normals directly.
    // Paper (Section 4.1) reports ~20% of Bullet results need this correction.
    if (mesh.getFaceRoll()[mesh.getHull().face(bestFaceIdx)].type != RollType::STABLE) {
        const Vector3& stuckFn = mesh.getHullGeom().faceNormals[mesh.getHull().face(bestFaceIdx)];
        Eigen::Vector3d stuckLocal(stuckFn.x, stuckFn.y, stuckFn.z);

        double bestStableDot = -std::numeric_limits<double>::infinity();
        int    bestStableIdx = bestFaceIdx;

        for (size_t i = 0; i < mesh._stableFaceNormals.size(); i++) {
            double d = stuckLocal.dot(mesh._stableFaceNormals[i]);
            if (d > bestStableDot) {
                bestStableDot = d;
                bestStableIdx = mesh._stableFaceIndices[i];
            }
        }
        bestFaceIdx = bestStableIdx;
    }

    return bestFaceIdx;
}

// ─────────────────────────────────────────────────────────────────────────────
// BulletSimulation::runTrials
//
// Main entry point.  Runs _p.numTrials independent drops, tallies which face
// lands down, writes FaceData<FaceResult> back onto mesh, and returns a flat
// vector of results sorted by face index.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<FaceResult> BulletSimulation::runTrials(Mesh& mesh)
{
    const int nFaces = static_cast<int>(mesh.getHull().nFaces());

    // Per-face landing counters
    std::vector<int> groupCount(mesh._numGroups, 0);

    std::mt19937 rng(std::random_device{}());
    auto t0 = std::chrono::steady_clock::now();

    for (int trial = 0; trial < _p.numTrials; ++trial) {
        btQuaternion rot = randomUnitQuaternion(rng);
        World w = buildWorld(mesh, rot);

        // Step until the body is asleep or we hit the safety cap
        int steps = 0;
        while (!bodyIsAsleep(w.body) && steps < _p.maxStepsPerTrial) {
            w.world->stepSimulation(
                static_cast<btScalar>(_p.fixedTimeStep),
                _p.maxSubSteps,
                static_cast<btScalar>(_p.fixedTimeStep));
            ++steps;
        }

        // Read final transform
        btTransform finalT;
        w.body->getMotionState()->getWorldTransform(finalT);

        int faceIdx = identifyRestingFace(mesh, finalT);
        int group = mesh._faceGroup[faceIdx];
        if (group >= 0) groupCount[group]++;

        destroyWorld(w);

        // Progress every 50 trials
        if ((trial + 1) % 50 == 0 || trial + 1 == _p.numTrials) {
            auto elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - t0).count();
            std::cout << "  Trial " << (trial + 1) << " / " << _p.numTrials
                      << "  (" << std::fixed << std::setprecision(1)
                      << elapsed << " s)\n";
        }
    }

    // ── Build result vector ──────────────────────────────────────────────────
    std::vector<FaceResult> results(nFaces);
    for (int i = 0; i < nFaces; ++i) {
        int group = mesh._faceGroup[i];
        results[i].faceIndex   = static_cast<size_t>(i);
        results[i].landCount   = (group >= 0) ? groupCount[group] : 0;
        results[i].probability = (group >= 0)
                                     ? static_cast<double>(groupCount[group]) / _p.numTrials
                                     : 0.0;
    }

    // ── Write back onto the Mesh so visualisation can use it ─────────────────
    mesh.setFaceResults(results);

    return results;
}


