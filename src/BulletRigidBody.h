#ifndef NUKEBULLET_BULLETRIGIDBODY_H
#define NUKEBULLET_BULLETRIGIDBODY_H


class BulletRigidBody
{
    btRigidBody* fallRigidBody;
    btDefaultMotionState* fallMotionState;
    btScalar mass;
    btCollisionShape* fallShape;
    btVector3 fallInertia;

public:
    BulletRigidBody();
    ~BulletRigidBody();

    void setup();
    void teardown();

    void reset();

    void addToWorld(btDiscreteDynamicsWorld* world);
    bool getMove(DD::Image::Matrix4& move);
};

#endif
