/*
 * Projectile motion: BulletPhysics library + OpenGL/GLUT.
 *
 * Simulation uses PhysicsWorld with forces::Gravity and PhysicsContext gravity
 * (constants::GRAVITY) + RK4. The library uses
 * a Y-up world: horizontal (x,z), vertical y. Assignment velocity components
 * (vx, vy horizontal in XY, vz up) map to library velocity as (vx, vz, vy).
 *
 * Render mapping matches the assignment camera transform relative to Z-up
 * physics: x_cam = z_lib, y_cam = y_lib, z_cam = x_lib.
 */

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION 1
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "Constants.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/forces/Gravity.h"
#include "builtin/bodies/RigidBody.h"
#include "math/Angles.h"
#include "math/Integrator.h"
#include "math/Vec3.h"

namespace {
constexpr double kDt = 1.0 / 60.0;
constexpr int kTimerMs = 1000 / 60;

inline double gravityScalar()
{
    return std::abs(BulletPhysics::constants::GRAVITY.y);
}

struct Params {
    double z0 = 0.0;
    double v0 = 0.0;
    double alphaDeg = 0.0;
    double phiDeg = 0.0;
    double radius = 0.1;
};

Params gParams;
std::unique_ptr<BulletPhysics::ballistics::external::PhysicsWorld> gWorld;
BulletPhysics::builtin::bodies::RigidBody gBody;
BulletPhysics::math::RK4Integrator gIntegrator;
std::vector<BulletPhysics::math::Vec3> gTrajectory;
bool gRunning = true;

BulletPhysics::math::Vec3 mapToRender(const BulletPhysics::math::Vec3& p)
{
    return {p.z, p.y, p.x};
}

void printAnalyticalResults(const Params& p)
{
    const double g = gravityScalar();
    const double a = BulletPhysics::math::deg2rad(p.alphaDeg);
    const double ph = BulletPhysics::math::deg2rad(p.phiDeg);

    const double vx = p.v0 * std::cos(a) * std::cos(ph);
    const double vy = p.v0 * std::cos(a) * std::sin(ph);
    const double vz = p.v0 * std::sin(a);

    const double tD = (vz + std::sqrt(vz * vz + 2.0 * g * p.z0)) / g;
    const double xmax = vx * tD;
    const double ymax = vy * tD;
    const double zmax = p.z0 + (vz * vz) / (2.0 * g);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== Analytical results ===\n";
    std::cout << "Initial velocity components (assignment XY ground, Z up):\n";
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

void initPhysics(const Params& p)
{
    using namespace BulletPhysics::ballistics::external;
    using namespace BulletPhysics::ballistics::external::forces;

    // PhysicsWorld::applyForces resets context; gravity comes from constants::GRAVITY
    // (see PhysicsContext::reset). Standard force: Gravity (IForce), not IEnvironment.
    gWorld = std::make_unique<PhysicsWorld>();
    gWorld->addForce(std::make_unique<Gravity>());

    gBody = BulletPhysics::builtin::bodies::RigidBody{};
    gBody.setMass(1.0);

    const double a = BulletPhysics::math::deg2rad(p.alphaDeg);
    const double ph = BulletPhysics::math::deg2rad(p.phiDeg);
    const double vx = p.v0 * std::cos(a) * std::cos(ph);
    const double vy = p.v0 * std::cos(a) * std::sin(ph);
    const double vz = p.v0 * std::sin(a);

    gBody.setPosition({0.0, p.z0, 0.0});
    gBody.setVelocity({vx, vz, vy});

    gRunning = true;
    gTrajectory.clear();
    gTrajectory.push_back(gBody.getPosition());
}

void cleanupPhysics()
{
    if (gWorld) {
        gWorld->clear();
        gWorld.reset();
    }
}

void drawAxes(float length = 20.0f)
{
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

void drawGround(float size = 50.0f, float step = 2.0f)
{
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

void drawTrajectory()
{
    glColor3f(1.0f, 1.0f, 0.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const BulletPhysics::math::Vec3& p : gTrajectory) {
        const BulletPhysics::math::Vec3 rp = mapToRender(p);
        glVertex3f(static_cast<GLfloat>(rp.x), static_cast<GLfloat>(rp.y), static_cast<GLfloat>(rp.z));
    }
    glEnd();
    glLineWidth(1.0f);
}

void drawSphere()
{
    const BulletPhysics::math::Vec3 rp = mapToRender(gBody.getPosition());
    glPushMatrix();
    glTranslatef(static_cast<GLfloat>(rp.x), static_cast<GLfloat>(rp.y), static_cast<GLfloat>(rp.z));
    glColor3f(1.0f, 0.3f, 0.3f);
    glutWireSphere(static_cast<double>(gParams.radius), 18, 18);
    glPopMatrix();
}

void display()
{
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

void timer(int)
{
    if (gRunning && gWorld) {
        gIntegrator.step(gBody, gWorld.get(), kDt);

        gTrajectory.push_back(gBody.getPosition());

        const BulletPhysics::math::Vec3 pos = gBody.getPosition();
        const BulletPhysics::math::Vec3 vel = gBody.getVelocity();
        if (pos.y <= gParams.radius && vel.y <= 0.0) {
            gRunning = false;
            gBody.setVelocity({0.0, 0.0, 0.0});
        }
    }

    glutPostRedisplay();
    glutTimerFunc(kTimerMs, timer, 0);
}

void reshape(int w, int h)
{
    if (h == 0) {
        h = 1;
    }

    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, static_cast<double>(w) / static_cast<double>(h), 0.1, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int, int)
{
    if (key == 27) {
        cleanupPhysics();
        std::exit(0);
    }
}

bool parseArgs(int argc, char** argv, Params& p)
{
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

int main(int argc, char** argv)
{
    if (!parseArgs(argc, argv, gParams)) {
        return 1;
    }

    printAnalyticalResults(gParams);
    initPhysics(gParams);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1100, 700);
    glutCreateWindow("3D Projectile (BulletPhysics lib + GLUT)");

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
