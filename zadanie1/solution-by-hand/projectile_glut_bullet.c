#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

float pi = 3.14f;

const float g = 9.81f;
const int timerInterval = 25;

float z0, v0, alphaDeg, phiDeg, radiusBall;
float alphaRad, phiRad;
float v0x, v0y, v0z;

float tD;
float xmaxRange, ymaxRange, zmaxRange;

float currentTime = 0.0f;
float dt = 0.025f;
int finished = 0;

float sceneSize = 10.0f;

float posX, posY, posZ;

void compute()
{
    alphaRad = alphaDeg * pi / 180.0f;
    phiRad = phiDeg * pi / 180.0f;

    v0x = v0 * cosf(alphaRad) * cosf(phiRad);
    v0y = v0 * cosf(alphaRad) * sinf(phiRad);
    v0z = v0 * sinf(alphaRad);

    tD = (v0z + sqrtf(v0z * v0z + 2.0f * g * z0)) / g;

    xmaxRange = v0x * tD;
    ymaxRange = v0y * tD;

    if (v0z > 0)
        zmaxRange = z0 + (v0z * v0z) / (2.0f * g);
    else
        zmaxRange = z0;

    float max = xmaxRange;
    if (ymaxRange > max) max = ymaxRange;
    if (zmaxRange > max) max = zmaxRange;

    sceneSize = max + 5.0f;

    posX = 0;
    posY = 0;
    posZ = z0;
}

void drawAxes()
{
    glBegin(GL_LINES);

    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(sceneSize, 0, 0);

    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, sceneSize, 0);

    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, sceneSize);

    glEnd();
}

void drawTrajectory()
{
    glColor3f(0.7, 0.7, 0.7);
    glBegin(GL_LINE_STRIP);

    for (int i = 0; i <= 200; i++)
    {
        float t = tD * i / 200.0f;

        float x = v0x * t;
        float y = v0y * t;
        float z = z0 + v0z * t - 0.5f * g * t * t;

        if (z < 0) z = 0;

        float xk = y;
        float yk = z;
        float zk = x;

        glVertex3f(xk, yk, zk);
    }

    glEnd();
}

void drawBall()
{
    float xk = posY;
    float yk = posZ;
    float zk = posX;

    glColor3f(0, 0, 0);

    glPushMatrix();
    glTranslatef(xk, yk, zk);
    glutWireSphere(radiusBall, 20, 20);
    glPopMatrix();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glRotatef(20.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(-55.0f, 0.0f, 1.0f, 0.0f);

    glTranslatef(
        -ymaxRange / 2.0f,
        -zmaxRange / 2.0f,
        -xmaxRange / 2.0f
    );

    drawAxes();
    drawTrajectory();
    drawBall();

    glutSwapBuffers();
}

void reshape(int w, int h)
{
    if (h == 0) h = 1;

    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float aspect = (float)w / h;

    glOrtho(
        -sceneSize * aspect, sceneSize * aspect,
        -sceneSize, sceneSize,
        -sceneSize, sceneSize
    );
}

void timer(int v)
{
    if (!finished)
    {
        currentTime += dt;

        if (currentTime >= tD)
        {
            currentTime = tD;
            finished = 1;
        }

        posX = v0x * currentTime;
        posY = v0y * currentTime;
        posZ = z0 + v0z * currentTime - 0.5f * g * currentTime * currentTime;

        if (posZ < 0)
        {
            posZ = 0;
            finished = 1;
        }

        glutPostRedisplay();
        glutTimerFunc(timerInterval, timer, 0);
    }
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        printf("Usage: %s z0 v0 alpha phi r\n", argv[0]);
        return 1;
    }

    z0 = atof(argv[1]);
    v0 = atof(argv[2]);
    alphaDeg = atof(argv[3]);
    phiDeg = atof(argv[4]);
    radiusBall = atof(argv[5]);

    compute();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 700);
    glutCreateWindow("Visualization");

    glEnable(GL_DEPTH_TEST);
    glClearColor(1, 1, 1, 1);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(timerInterval, timer, 0);

    printf("RESULTS:\n");
    printf("tD = %f\n", tD);
    printf("xmax = %f\n", xmaxRange);
    printf("ymax = %f\n", ymaxRange);
    printf("zmax = %f\n", zmaxRange);

    glutMainLoop();

    return 0;
}