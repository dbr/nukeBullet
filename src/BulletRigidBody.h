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


class BulletRigidBodyNode : public DD::Image::ModifyGeo
{
private:

    BulletRigidBody *thisobj;

public:

    BulletRigidBodyNode(Node* node);
    ~BulletRigidBodyNode();

    int minimum_inputs() const { return 1; }
    int maximum_inputs() const { return 1024; }
    bool test_input(int input, DD::Image::Op* op) const;

    void knobs(DD::Image::Knob_Callback f);

    void _validate(bool for_real);
    void get_geometry_hash();
    void modify_geometry(int, DD::Image::Scene&, DD::Image::GeometryList&);

    const char* Class() const { return "BulletRigidBodyNode"; }
    const char* node_help() const { return "Connects to Geo, then plugins into BulletSolver"; }
    const char * node_shape() const { return "[/"; }

    static const DD::Image::Op::Description description;

    BulletRigidBody* getobj();
};

#endif
