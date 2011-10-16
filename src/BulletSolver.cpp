#include <vector>

#include "DDImage/GeoOp.h"
#include "DDImage/SourceGeo.h"
#include "DDImage/ModifyGeo.h"
#include "DDImage/Knob.h"
#include "DDImage/Knobs.h"
#include "DDImage/Matrix4.h"
#include <btBulletDynamicsCommon.h>

#include "BulletSolver.h"
#include "BulletRigidBody.h"

static const char* const HELP = "Bullet World into which stuff is connected.";


Solveamajig::Solveamajig()
{
    std::cerr << "Solveamajig ctor\n";
    setup();
}

void Solveamajig::setup()
{
    // Build the broadphase
    broadphase = new btDbvtBroadphase();

    // Set up the collision configuration and dispatcher
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    // The actual physics solver
    solver = new btSequentialImpulseConstraintSolver;

    // The world.
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

    // Create test objects
    groundShape = new btStaticPlaneShape(btVector3(0,1,0), 1);
    groundMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));

    btRigidBody::btRigidBodyConstructionInfo
        groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(15,0,0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

    dynamicsWorld->addRigidBody(groundRigidBody);

    std::vector<BulletRigidBody*>::iterator iter;
    for(iter = m_objects.begin(); iter != m_objects.end(); ++iter)
    {
        (*iter)->setup();
    }

    m_sim_prev_frame = 0;
}

void Solveamajig::addObject(BulletRigidBody* obj)
{
    std::cerr << "Solveamajig::addObject\n";
    m_objects.push_back(obj);
    obj->addToWorld(dynamicsWorld);
}

Solveamajig::~Solveamajig()
{
    std::cerr << "Solveamajig dtor\n";
    teardown();
}

void Solveamajig::teardown()
{
    std::vector<BulletRigidBody*>::iterator iter;
    for(iter = m_objects.begin(); iter != m_objects.end(); ++iter)
    {
        (*iter)->teardown();
    }

    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
}

void Solveamajig::step(int frame)
{
    if(frame == m_sim_prev_frame+1)
    {
        std::cerr << "Sim'ing next frame (from " << m_sim_prev_frame << " to " << frame << ")\n";
        dynamicsWorld->stepSimulation(1/24.f, kSubsteps);
        m_sim_prev_frame = frame;
        return;
    }
    else if(m_sim_prev_frame == frame)
    {
        std::cerr << "Frame not changed\n";
        return;
    }
    else if(frame < m_sim_prev_frame)
    {
        std::cerr << "Backwards step!\n";
        resetsim();
        m_sim_prev_frame = 1;
    }

    // If sim has reset, or stepping more than 1 frame, sim up to
    // current frame. Ineffecient
    if(frame > m_sim_prev_frame)
    {
        std::cerr << "Fast forwarding sim from frame " << m_sim_prev_frame << " to " << frame << "\n";
        while(m_sim_prev_frame < frame)
        {
            dynamicsWorld->stepSimulation(1/24.f, kSubsteps);
            m_sim_prev_frame++;
        }
    }
}

void Solveamajig::resetsim()
{
    teardown();
    setup();

    std::vector<BulletRigidBody*>::iterator iter;
    for(iter = m_objects.begin(); iter != m_objects.end(); ++iter)
    {
        (*iter)->addToWorld(dynamicsWorld);
    }
}


BulletRigidBody::BulletRigidBody()
{
    std::cerr << "BulletObject ctor\n";
    setup();
}

void BulletRigidBody::setup()
{
    mass = 1.0;

    // Radius
    fallShape = new btSphereShape(1);

    // Default position and rotation
    fallMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0,0,0,1), btVector3(0,20,0)));

    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);

    fallRigidBody = new btRigidBody(fallRigidBodyCI);
}

BulletRigidBody::~BulletRigidBody()
{
    std::cerr << "BulletRigidBody dtor\n";
    teardown();
}
void BulletRigidBody::teardown()
{
    delete fallRigidBody;
    delete fallMotionState;
    delete fallShape;
}

void BulletRigidBody::addToWorld(btDiscreteDynamicsWorld* world)
{
    std::cerr << "BulletRigidBody::addToWorld\n";
    world->addRigidBody(fallRigidBody);
}

void BulletRigidBody::reset()
{
    std::cerr << "BulletRigidBody::reset\n";
    teardown();
    setup();
}

bool BulletRigidBody::getMove(DD::Image::Matrix4& move)
{
    std::cerr << "Getting world transform for fallRigidBody\n";
    btTransform trans;
    fallRigidBody->getMotionState()->getWorldTransform(trans);

    move.translate(
        trans.getOrigin().x(),
        trans.getOrigin().y(),
        trans.getOrigin().z());

    btQuaternion rot = trans.getRotation();
    move.rotate(rot.x(), rot.y(), rot.z(), rot.w());
    return true;
}



class BulletSolver : public DD::Image::ModifyGeo
{
private:

    btBroadphaseInterface*  m_broadphase;
    btCollisionDispatcher*  m_dispatcher;
    btConstraintSolver*     m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;

    Solveamajig *bsolve;
    int frame;

    BulletRigidBody *testobj;

public:

    int minimum_inputs() const { return 1; }
    int maximum_inputs() const { return 1024; }

    bool test_input(int input, DD::Image::Op* op) const
    {
        if(strcmp(op->Class(), "NullGeo") == 0)
        {
            // Accept the NullGeo input, when no real node is
            // connected, otherwise modify_geometry dies
            return true;
        }
        bool is_thing = dynamic_cast<DD::Image::SourceGeo*>(op) != 0;
        return is_thing;
    }

    void knobs(DD::Image::Knob_Callback f)
    {
        DD::Image::ModifyGeo::knobs(f);
        DD::Image::Int_knob(f, &frame, "sim_frame", "sim frame");
    }

    BulletSolver(Node* node) : ModifyGeo(node)
    {
        std::cerr << "BulletSolver ctor\n";
        bsolve = new Solveamajig();

        std::cerr << "Making sphere thing\n";
        testobj = new BulletRigidBody();

        std::cerr << "Adding sphere to world\n";
        bsolve->addObject(testobj);
        frame = 0.0;
    }

    ~BulletSolver()
    {
        std::cerr << "BulletSolver dtor\n";
        delete bsolve;
    }

    void _validate(bool for_real);
    void get_geometry_hash();
    void modify_geometry(int, DD::Image::Scene&, DD::Image::GeometryList&);

    const char* Class() const { return "BulletSolver"; }
    const char* node_help() const { return "Shooot me"; }
    const char * node_shape() const { return "[)"; }

    static const DD::Image::Op::Description description;

    std::string testthingy()
    {
        return "Testing \\o/";
    }
};


void BulletSolver::get_geometry_hash()
{
    // Get all hashes up-to-date
    ModifyGeo::get_geometry_hash();

    // Values that change the Local to World matrix:
    geo_hash[DD::Image::Group_Matrix].append(frame);
}

void BulletSolver::_validate(bool for_real)
{
    ModifyGeo::_validate(for_real);

    // TODO: Use this instead of the sim_frame knob. sim_frame was
    // needed to make _validate etc be called on frame change, but
    // there must be a better way

    //frame = outputContext().frame();
}

void BulletSolver::modify_geometry(int obj, DD::Image::Scene& scene, DD::Image::GeometryList& geolist)
{
    std::cerr << "BulletSolver::modify_geometry\n";

    /*
    BulletSolver* bw = dynamic_cast<BulletSolver*>(input(1));
    if(bw != 0)
    {
        std::cerr << "!\n";
        std::cerr << bw->testthingy() << "\n";
        std::cerr << "Yay\n";
    }
    else
    {
        std::cerr << "Wrong input type on B\n";
    }
    */

    std::cerr << "Stepping sim to " << frame << "\n";
    bsolve->step(frame);
    
    if(!testobj->getMove(geolist[obj].matrix))
    {
        error("Something broke, uhoh. No idea what.");
    }
    std::cerr << geolist[obj].matrix << "\n";
}

static DD::Image::Op* buildBulletSolver(Node* node)
{
    return new BulletSolver(node);
}
const DD::Image::Op::Description BulletSolver::description("BulletSolver", buildBulletSolver);
