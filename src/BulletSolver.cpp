#include "DDImage/GeoOp.h"
#include "DDImage/SourceGeo.h"
#include "DDImage/ModifyGeo.h"
#include "DDImage/Knob.h"
#include "DDImage/Knobs.h"
#include "DDImage/Matrix4.h"
#include <btBulletDynamicsCommon.h>

static const char* const HELP = "Bullet World into which stuff is connected.";


class Solveamajig
{
private:
    btBroadphaseInterface* broadphase;
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;

    int m_sim_prev_frame;

    // Test object
    btRigidBody* fallRigidBody;
    btCollisionShape* groundShape;
    btCollisionShape* fallShape;
    btDefaultMotionState* groundMotionState;
    btDefaultMotionState* fallMotionState;
    btScalar mass;
    btVector3 fallInertia;
    static const int kSubsteps = 100;

public:
    Solveamajig()
    {
        std::cerr << "Solveamajig ctor\n";
        setup();
    }

    void setup()
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
        fallShape = new btSphereShape(1);
        groundMotionState =
            new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));

        btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0,0,0));
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

        dynamicsWorld->addRigidBody(groundRigidBody);
        fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,20,0)));

        mass = 0.2;
        fallInertia = btVector3(0,0,0);
        fallShape->calculateLocalInertia(mass, fallInertia);

        btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
        fallRigidBody = new btRigidBody(fallRigidBodyCI);
        dynamicsWorld->addRigidBody(fallRigidBody);

        m_sim_prev_frame = 0;
    }

    ~Solveamajig()
    {
        std::cerr << "Solveamajig dtor\n";
        teardown();
    }
    void teardown()
    {
        delete fallRigidBody;
        delete dynamicsWorld;
        delete solver;
        delete dispatcher;
        delete collisionConfiguration;
        delete broadphase;
    }

    void step(int frame)
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
            std::cerr << "Backwards step!";
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

    void resetsim()
    {
        teardown();
        setup();
    }

    bool getSpherePosition(DD::Image::Matrix4& themove)
    {
        std::cerr << "Getting world transform for fallRigidBody\n";
        btTransform trans;
        fallRigidBody->getMotionState()->getWorldTransform(trans);

        std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;

        themove.translate(
            trans.getOrigin().x(),
            trans.getOrigin().y(),
            trans.getOrigin().z());


        btQuaternion rot = trans.getRotation();
        themove.rotate(rot.x(), rot.y(), rot.z(), rot.w());
        return true;
    }
};

class BulletSolver : public DD::Image::ModifyGeo
{
private:

    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btBroadphaseInterface*  m_broadphase;
    btCollisionDispatcher*  m_dispatcher;
    btConstraintSolver*     m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;

    Solveamajig *bsolve;
    int frame;

public:

    int minimum_inputs() const { return 1; }
    int maximum_inputs() const { return 1024; }

    bool test_input(int input, DD::Image::Op* op) const
    {
        std::cerr << "Testing input " << op->Class() << "\n";
        if(strcmp(op->Class(), "NullGeo") == 0)
        {
            // Accept the NullGeo input, when no real node is
            // connected, otherwise modify_geometry dies
            return true;
        }
        bool is_thing = dynamic_cast<DD::Image::SourceGeo*>(op) != 0;
        std::cerr << "test input " << is_thing << "\n";
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
    std::cerr << "BulletSolver::get_geometry_hash\n";

    // Get all hashes up-to-date
    ModifyGeo::get_geometry_hash();

    std::cerr << "Appending frame\n";
    // Values that change the Local to World matrix:
    geo_hash[DD::Image::Group_Matrix].append(frame);
    std::cerr << "End get_geometry_hash\n";
}

void BulletSolver::_validate(bool for_real)
{
    std::cerr << "BulletSolver::_validate\n";
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
    if(!bsolve->getSpherePosition(geolist[obj].matrix)){
        error("Something broke, uhoh. No idea what.");
    }
    std::cerr << geolist[obj].matrix << "\n";
}

static DD::Image::Op* buildBulletSolver(Node* node)
{
    return new BulletSolver(node);
}
const DD::Image::Op::Description BulletSolver::description("BulletSolver", buildBulletSolver);
