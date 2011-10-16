#ifndef NUKEBULLET_BULLETSOLVER_H
#define NUKEBULLET_BULLETSOLVER_H


#include "BulletRigidBody.h"

class Solveamajig
{
private:
    btBroadphaseInterface* broadphase;
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;

    std::vector<BulletRigidBody*> m_objects;

    int m_sim_prev_frame;

    // Test object
    btCollisionShape* groundShape;
    btDefaultMotionState* groundMotionState;
    static const int kSubsteps = 100;

public:
    Solveamajig();
    void setup();

    ~Solveamajig();
    void teardown();

    void addObject(BulletRigidBody* obj);

    void step(int frame);
    void resetsim();
};

#endif
