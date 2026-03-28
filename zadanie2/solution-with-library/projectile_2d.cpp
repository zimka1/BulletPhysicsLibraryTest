/*
 * Два тела в 2D: как visualization_table_graph.c, но физика шагами через BulletPhysics
 * (RigidBody + PhysicsWorld + Gravity + RK4). Без gnuplot — только окно GLUT и output.txt.
 */

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

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
#include "math/Integrator.h"
#include "math/Vec3.h"

namespace {

namespace ext = BulletPhysics::ballistics::external;
namespace forces = ext::forces;

using RigidBody = BulletPhysics::builtin::bodies::RigidBody;
using RK4 = BulletPhysics::math::RK4Integrator;
using Vec3 = BulletPhysics::math::Vec3;

inline double gVal()
{
    return std::abs(BulletPhysics::constants::GRAVITY.y);
}

// Как в .c
const int timeStep = 16;
const float dt = 0.01f;

double Y0 = 0.0;
double v0 = 0.0;
double alphaDeg = 0.0;
double d = 0.0;
double betaDeg = 0.0;
double size = 0.0;

double alpha = 0.0;
double beta = 0.0;

double xB0 = 0.0;
double yB0 = 0.0;

float t = 0.0f;
int motionFinished = 0;

std::unique_ptr<ext::PhysicsWorld> gWorld;
RigidBody gBodyA;
RigidBody gBodyB;
RK4 gIntegrator;

void drawSquare(double halfExtent)
{
    glBegin(GL_QUADS);
    glVertex2f(static_cast<GLfloat>(-halfExtent), static_cast<GLfloat>(-halfExtent));
    glVertex2f(static_cast<GLfloat>(halfExtent), static_cast<GLfloat>(-halfExtent));
    glVertex2f(static_cast<GLfloat>(halfExtent), static_cast<GLfloat>(halfExtent));
    glVertex2f(static_cast<GLfloat>(-halfExtent), static_cast<GLfloat>(halfExtent));
    glEnd();
}

void initPhysics()
{
    gWorld = std::make_unique<ext::PhysicsWorld>();
    gWorld->addForce(std::make_unique<forces::Gravity>());

    const double v0x = v0 * std::cos(alpha);
    const double v0y = v0 * std::sin(alpha);

    gBodyA = RigidBody{};
    gBodyA.setMass(1.0);
    gBodyA.setPosition({0.0, Y0, 0.0});
    gBodyA.setVelocity({v0x, v0y, 0.0});

    gBodyB = RigidBody{};
    gBodyB.setMass(1.0);
    gBodyB.setPosition({xB0, yB0, 0.0});
    gBodyB.setVelocity({0.0, 0.0, 0.0});

    t = 0.0f;
    motionFinished = 0;
}

void update(int value)
{
    if (motionFinished) {
        return;
    }

    gIntegrator.step(gBodyA, gWorld.get(), static_cast<double>(dt));
    gIntegrator.step(gBodyB, gWorld.get(), static_cast<double>(dt));
    t += dt;

    const Vec3 pa = gBodyA.getPosition();
    const Vec3 pb = gBodyB.getPosition();
    const double xA = pa.x;
    const double yA = pa.y;
    const double yB = pb.y;

    if (yA <= 0.0 && yB <= 0.0) {
        motionFinished = 1;
    }
    if (std::fabs(xA - xB0) < size && std::fabs(yA - yB) < size) {
        motionFinished = 1;
    }

    glutPostRedisplay();

    if (!motionFinished) {
        glutTimerFunc(timeStep, update, value + 1);
    }
}

void reshape(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    const double worldW = d + 5.0;
    const double worldH = std::fmax(Y0 + 5.0, yB0 + 5.0);

    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    const double worldAspect = worldW / worldH;

    if (aspect > worldAspect) {
        const double newW = worldH * aspect;
        gluOrtho2D(0.0, newW, 0.0, worldH);
    } else {
        const double newH = worldW / aspect;
        gluOrtho2D(0.0, worldW, 0.0, newH);
    }

    glMatrixMode(GL_MODELVIEW);
}

void drawScene()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    const Vec3 pa = gBodyA.getPosition();
    const Vec3 pb = gBodyB.getPosition();

    glPushMatrix();
    glTranslatef(static_cast<GLfloat>(pa.x), static_cast<GLfloat>(pa.y), 0.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    drawSquare(size);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(static_cast<GLfloat>(xB0), static_cast<GLfloat>(pb.y), 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    drawSquare(size);
    glPopMatrix();

    glutSwapBuffers();
}

int main(int argc, char** argv)
{
    if (argc != 7) {
        std::cerr << "Usage:\n" << argv[0] << " y0 v0 alpha d beta size\n";
        return 1;
    }

    Y0       = std::atof(argv[1]);
    v0      = std::atof(argv[2]);
    alphaDeg = std::atof(argv[3]);
    d       = std::atof(argv[4]);
    betaDeg = std::atof(argv[5]);
    size    = std::atof(argv[6]);

    alpha = alphaDeg * M_PI / 180.0;
    beta  = betaDeg * M_PI / 180.0;

    xB0 = d * std::cos(beta);
    yB0 = d * std::sin(beta);

    initPhysics();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(900, 600);
    glutCreateWindow("Visualization (BulletPhysics)");

    glClearColor(0.1f, 0.3f, 0.3f, 1.0f);

    glutDisplayFunc(drawScene);
    glutReshapeFunc(reshape);
    glutTimerFunc(timeStep, update, 0);

    glutMainLoop();

    return 0;
}
