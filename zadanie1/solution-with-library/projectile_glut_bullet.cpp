#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <btBulletDynamicsCommon.h>

namespace {
constexpr double kGravity = 9.81;
constexpr float kDt = 1.0f / 60.0f;
constexpr int kTimerMs = 1000 / 60;

struct Params {
    double z0 = 0.0;
    double v0 = 0.0;
    double alphaDeg = 0.0;
    double phiDeg = 0.0;
    double radius = 0.1;
};

struct BulletState {
    btDefaultCollisionConfiguration* collisionConfig = nullptr;
    btCollisionDispatcher* dispatcher = nullptr;
    btBroadphaseInterface* broadphase = nullptr;
    btSequentialImpulseConstraintSolver* solver = nullptr;
    btDiscreteDynamicsWorld* world = nullptr;

    btCollisionShape* groundShape = nullptr;
    btCollisionShape* sphereShape = nullptr;

    btRigidBody* groundBody = nullptr;
    btRigidBody* sphereBody = nullptr;

    std::vector<btCollisionShape*> shapes;
    std::vector<btRigidBody*> bodies;
};

Params gParams;
BulletState gBullet;
std::vector<btVector3> gTrajectory;
bool gRunning = true;

double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

btVector3 mapToRender(const btVector3& p) {
    // Bullet: (x, y, z), Render: (y, z, x)
    return btVector3(p.getY(), p.getZ(), p.getX());
}

void printAnalyticalResults(const Params& p) {
    const double a = degToRad(p.alphaDeg);
    const double ph = degToRad(p.phiDeg);

    const double vx = p.v0 * std::cos(a) * std::cos(ph);
    const double vy = p.v0 * std::cos(a) * std::sin(ph);
    const double vz = p.v0 * std::sin(a);

    const double tD = (vz + std::sqrt(vz * vz + 2.0 * kGravity * p.z0)) / kGravity;
    const double xmax = vx * tD;
    const double ymax = vy * tD;
    const double zmax = p.z0 + (vz * vz) / (2.0 * kGravity);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Analytical results ===\n";
    std::cout << "Initial velocity components:\n";
    std::cout << "  vx = " << vx << " m/s\n";
    std::cout << "  vy = " << vy << " m/s\n";
    std::cout << "  vz = " << vz << " m/s\n\n";
    std::cout << "Time of flight (tD): " << tD << " s\n";
    std::cout << "Maximum values:\n";
    std::cout << "  xmax = " << xmax << " m\n";
    std::cout << "  ymax = " << ymax << " m\n";
    std::cout << "  zmax = " << zmax << " m\n";
    std::cout << "==========================\n\n";
}

btRigidBody* createRigidBody(float mass, const btTransform& transform, btCollisionShape* shape) {
    btVector3 localInertia(0, 0, 0);
    if (mass > 0.0f) {
        shape->calculateLocalInertia(mass, localInertia);
    }

    auto* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
    auto* body = new btRigidBody(rbInfo);
    gBullet.bodies.push_back(body);
    return body;
}

void initPhysics(const Params& p) {
    gBullet.collisionConfig = new btDefaultCollisionConfiguration();
    gBullet.dispatcher = new btCollisionDispatcher(gBullet.collisionConfig);
    gBullet.broadphase = new btDbvtBroadphase();
    gBullet.solver = new btSequentialImpulseConstraintSolver();
    gBullet.world = new btDiscreteDynamicsWorld(
        gBullet.dispatcher, gBullet.broadphase, gBullet.solver, gBullet.collisionConfig);
    gBullet.world->setGravity(btVector3(0, 0, -kGravity));

    gBullet.groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0.0);
    gBullet.shapes.push_back(gBullet.groundShape);
    btTransform groundTr;
    groundTr.setIdentity();
    groundTr.setOrigin(btVector3(0, 0, 0));
    gBullet.groundBody = createRigidBody(0.0f, groundTr, gBullet.groundShape);
    gBullet.world->addRigidBody(gBullet.groundBody);

    gBullet.sphereShape = new btSphereShape(static_cast<btScalar>(p.radius));
    gBullet.shapes.push_back(gBullet.sphereShape);
    btTransform sphereTr;
    sphereTr.setIdentity();
    sphereTr.setOrigin(btVector3(0, 0, static_cast<btScalar>(p.z0)));
    gBullet.sphereBody = createRigidBody(1.0f, sphereTr, gBullet.sphereShape);
    gBullet.sphereBody->setRestitution(0.0f);
    gBullet.sphereBody->setFriction(0.5f);
    gBullet.world->addRigidBody(gBullet.sphereBody);

    const double a = degToRad(p.alphaDeg);
    const double ph = degToRad(p.phiDeg);
    const btScalar vx = static_cast<btScalar>(p.v0 * std::cos(a) * std::cos(ph));
    const btScalar vy = static_cast<btScalar>(p.v0 * std::cos(a) * std::sin(ph));
    const btScalar vz = static_cast<btScalar>(p.v0 * std::sin(a));
    gBullet.sphereBody->setLinearVelocity(btVector3(vx, vy, vz));
    gBullet.sphereBody->activate(true);

    gTrajectory.clear();
    gTrajectory.push_back(gBullet.sphereBody->getWorldTransform().getOrigin());
}

void cleanupPhysics() {
    if (!gBullet.world) {
        return;
    }

    for (btRigidBody* body : gBullet.bodies) {
        if (body) {
            gBullet.world->removeRigidBody(body);
            delete body->getMotionState();
            delete body;
        }
    }
    gBullet.bodies.clear();

    for (btCollisionShape* shape : gBullet.shapes) {
        delete shape;
    }
    gBullet.shapes.clear();

    delete gBullet.world;
    delete gBullet.solver;
    delete gBullet.broadphase;
    delete gBullet.dispatcher;
    delete gBullet.collisionConfig;

    gBullet.world = nullptr;
    gBullet.solver = nullptr;
    gBullet.broadphase = nullptr;
    gBullet.dispatcher = nullptr;
    gBullet.collisionConfig = nullptr;
}

void drawAxes(float length = 20.0f) {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, length, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, length);
    glEnd();
    glLineWidth(1.0f);
}

void drawGround(float size = 50.0f, float step = 2.0f) {
    glColor3f(0.7f, 0.7f, 0.7f);
    glBegin(GL_LINES);
    for (float i = -size; i <= size; i += step) {
        glVertex3f(-size, 0.0f, i);
        glVertex3f(size, 0.0f, i);
        glVertex3f(i, 0.0f, -size);
        glVertex3f(i, 0.0f, size);
    }
    glEnd();
}

void drawTrajectory() {
    glColor3f(1.0f, 1.0f, 0.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const btVector3& p : gTrajectory) {
        const btVector3 rp = mapToRender(p);
        glVertex3f(rp.getX(), rp.getY(), rp.getZ());
    }
    glEnd();
    glLineWidth(1.0f);
}

void drawSphere() {
    btTransform tr;
    gBullet.sphereBody->getMotionState()->getWorldTransform(tr);
    const btVector3 p = tr.getOrigin();
    const btVector3 rp = mapToRender(p);

    glPushMatrix();
    glTranslatef(rp.getX(), rp.getY(), rp.getZ());
    glColor3f(1.0f, 0.3f, 0.3f);
    glutWireSphere(gParams.radius, 18, 18);
    glPopMatrix();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    gluLookAt(
        30.0, 20.0, 35.0,
        0.0, 5.0, 0.0,
        0.0, 1.0, 0.0);

    drawGround();
    drawAxes();
    drawTrajectory();
    drawSphere();
    glutSwapBuffers();
}

void timer(int) {
    if (gRunning && gBullet.world && gBullet.sphereBody) {
        gBullet.world->stepSimulation(kDt, 10, kDt);

        btTransform tr;
        gBullet.sphereBody->getMotionState()->getWorldTransform(tr);
        const btVector3 pos = tr.getOrigin();
        gTrajectory.push_back(pos);

        const btVector3 vel = gBullet.sphereBody->getLinearVelocity();
        if (pos.getZ() <= static_cast<btScalar>(gParams.radius) && vel.getZ() <= 0.0f) {
            gRunning = false;
            gBullet.sphereBody->setLinearVelocity(btVector3(0, 0, 0));
            gBullet.sphereBody->setAngularVelocity(btVector3(0, 0, 0));
        }
    }

    glutPostRedisplay();
    glutTimerFunc(kTimerMs, timer, 0);
}

void reshape(int w, int h) {
    if (h == 0) {
        h = 1;
    }

    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, static_cast<double>(w) / static_cast<double>(h), 0.1, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int, int) {
    if (key == 27) {
        cleanupPhysics();
        std::exit(0);
    }
}

bool parseArgs(int argc, char** argv, Params& p) {
    if (argc != 6) {
        std::cerr << "Usage: " << argv[0] << " <z0> <v0> <alpha_deg> <phi_deg> <r>\n";
        return false;
    }

    p.z0 = std::atof(argv[1]);
    p.v0 = std::atof(argv[2]);
    p.alphaDeg = std::atof(argv[3]);
    p.phiDeg = std::atof(argv[4]);
    p.radius = std::atof(argv[5]);

    if (p.z0 < 0.0) {
        std::cerr << "Error: z0 must be >= 0.\n";
        return false;
    }
    if (p.v0 < 0.0) {
        std::cerr << "Error: v0 must be >= 0.\n";
        return false;
    }
    if (p.alphaDeg < 0.0 || p.alphaDeg > 90.0) {
        std::cerr << "Error: alpha must be in [0, 90].\n";
        return false;
    }
    if (p.phiDeg < 0.0 || p.phiDeg > 90.0) {
        std::cerr << "Error: phi must be in [0, 90].\n";
        return false;
    }
    if (p.radius <= 0.0) {
        std::cerr << "Error: r must be > 0.\n";
        return false;
    }

    return true;
}
} // namespace

int main(int argc, char** argv) {
    if (!parseArgs(argc, argv, gParams)) {
        return 1;
    }

    printAnalyticalResults(gParams);
    initPhysics(gParams);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1100, 700);
    glutCreateWindow("3D Projectile Motion (Bullet + OpenGL/GLUT)");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(kTimerMs, timer, 0);

    atexit(cleanupPhysics);
    glutMainLoop();
    return 0;
}
