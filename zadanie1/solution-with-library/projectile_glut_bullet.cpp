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

namespace bp = BulletPhysics;
namespace ext = bp::ballistics::external;
namespace forces = ext::forces;

using Vec3 = bp::math::Vec3;
using RigidBody = bp::builtin::bodies::RigidBody;

constexpr double kDt = 1.0 / 60.0;
constexpr int kTimerMs = 1000 / 60;

inline double gravityScalar()
{
    return std::abs(bp::constants::GRAVITY.y);
}

struct Params {
    double z0 = 0.0;
    double v0 = 0.0;
    double alphaDeg = 0.0;
    double phiDeg = 0.0;
    double radius = 0.1;
};

/// Assignment frame: vx, vy in ground plane, vz up (m/s).
struct AssignmentVelocity {
    double vx, vy, vz;
};

inline AssignmentVelocity assignmentVelocity(const Params& p)
{
    const double a = bp::math::deg2rad(p.alphaDeg);
    const double ph = bp::math::deg2rad(p.phiDeg);
    const double ce = std::cos(a) * p.v0;
    return {ce * std::cos(ph), ce * std::sin(ph), p.v0 * std::sin(a)};
}

/** Maps library angle convention to assignment-oriented body velocity (x,y_up,z). */
inline void setRigidBodyVelocityForAssignment(RigidBody& body, const Params& p)
{
    body.setVelocityFromAngles(p.v0, p.alphaDeg, p.phiDeg);
    const Vec3 v = body.getVelocity();
    body.setVelocity({v.z, v.y, v.x});
}

struct SimState {
    Params params{};
    std::unique_ptr<ext::PhysicsWorld> world;
    RigidBody body{};
    bp::math::RK4Integrator integrator{};
    std::vector<Vec3> trajectory;
    bool running = true;
};

SimState gSim;

inline Vec3 mapToRender(const Vec3& p)
{
    return {p.z, p.y, p.x};
}

inline void glVertex3(const Vec3& v)
{
    glVertex3f(static_cast<GLfloat>(v.x), static_cast<GLfloat>(v.y), static_cast<GLfloat>(v.z));
}

void printAnalyticalResults(const Params& p)
{
    const auto [vx, vy, vz] = assignmentVelocity(p);
    const double g = gravityScalar();

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

void initSimulation(SimState& sim)
{
    const Params& p = sim.params;

    sim.world = std::make_unique<ext::PhysicsWorld>();
    sim.world->addForce(std::make_unique<forces::Gravity>());

    sim.body = RigidBody{};
    sim.body.setMass(1.0);
    sim.body.setPosition({0.0, p.z0, 0.0});
    setRigidBodyVelocityForAssignment(sim.body, p);

    sim.running = true;
    sim.trajectory.clear();
    sim.trajectory.push_back(sim.body.getPosition());
}

void cleanupSimulation(SimState& sim)
{
    if (sim.world) {
        sim.world->clear();
        sim.world.reset();
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

void drawTrajectory(const SimState& sim)
{
    glColor3f(1.0f, 1.0f, 0.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const Vec3& p : sim.trajectory) {
        glVertex3(mapToRender(p));
    }
    glEnd();
    glLineWidth(1.0f);
}

void drawSphere(const SimState& sim)
{
    const Vec3 rp = mapToRender(sim.body.getPosition());
    glPushMatrix();
    glTranslatef(static_cast<GLfloat>(rp.x), static_cast<GLfloat>(rp.y), static_cast<GLfloat>(rp.z));
    glColor3f(1.0f, 0.3f, 0.3f);
    glutWireSphere(sim.params.radius, 18, 18);
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
    drawTrajectory(gSim);
    drawSphere(gSim);
    glutSwapBuffers();
}

void timer(int)
{
    if (gSim.running && gSim.world) {
        gSim.integrator.step(gSim.body, gSim.world.get(), kDt);
        gSim.trajectory.push_back(gSim.body.getPosition());

        const Vec3 pos = gSim.body.getPosition();
        const Vec3 vel = gSim.body.getVelocity();
        if (pos.y <= gSim.params.radius && vel.y <= 0.0) {
            gSim.running = false;
            gSim.body.setVelocity({0.0, 0.0, 0.0});
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
    if (!parseArgs(argc, argv, gSim.params)) {
        return 1;
    }

    printAnalyticalResults(gSim.params);
    initSimulation(gSim);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1100, 700);
    glutCreateWindow("3D Projectile (BulletPhysics lib + GLUT)");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(kTimerMs, timer, 0);

    atexit([] { cleanupSimulation(gSim); });
    glutMainLoop();
    return 0;
}
