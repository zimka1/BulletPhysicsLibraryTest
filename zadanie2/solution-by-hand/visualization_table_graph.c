#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

const int timeStep = 16;
const float dt = 0.01f;
const double g = 9.80665;

float t = 0.0f;
int motionFinished = 0;

double Y0, v0, alphaDeg, d, betaDeg, size;
double alpha, beta;

double xB0, yB0;

void calculate(double* xA, double* yA, double* yB)
{
    *xA = v0 * cos(alpha) * t;
    *yA = Y0 + v0 * sin(alpha) * t - 0.5 * g * t * t;
    *yB = yB0 - 0.5 * g * t * t;
}

void drawSquare(double size)
{
    glBegin(GL_QUADS);
        glVertex2f(-size, -size);
        glVertex2f(size, -size);
        glVertex2f(size, size);
        glVertex2f(-size, size);
    glEnd();
}

void update(int value)
{
    if (motionFinished) return;

    t += dt;

    double xA, yA, yB;
    calculate(&xA, &yA, &yB);

    if (yA <= 0.0 && yB <= 0.0)
        motionFinished = 1;

    if (fabs(xA - xB0) < size && fabs(yA - yB) < size)
        motionFinished = 1;

    glutPostRedisplay();

    if (!motionFinished)
        glutTimerFunc(timeStep, update, value + 1);
}

void reshape(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    double worldW = d + 5;
    double worldH = fmax(Y0 + 5, yB0 + 5);

    double aspect = (double)width / height;
    double worldAspect = worldW / worldH;

    if (aspect > worldAspect) {
        double newW = worldH * aspect;
        gluOrtho2D(0, newW, 0, worldH);
    } else {
        double newH = worldW / aspect;
        gluOrtho2D(0, worldW, 0, newH);
    }
}

void drawScene()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    double xA, yA, yB;
    calculate(&xA, &yA, &yB);

    glPushMatrix();
        glTranslatef(xA, yA, 0);
        glColor3f(1, 0, 0);
        drawSquare(size);
    glPopMatrix();

    glPushMatrix();
        glTranslatef(xB0, yB, 0);
        glColor3f(0, 0, 1);
        drawSquare(size);
    glPopMatrix();

    glutSwapBuffers();
}

int main(int argc, char** argv)
{
    if (argc != 7) {
        printf("Usage:\n");
        printf("./du3 y0 v0 alpha d beta size\n");
        return 1;
    }

    Y0 = atof(argv[1]);
    v0 = atof(argv[2]);
    alphaDeg = atof(argv[3]);
    d = atof(argv[4]);
    betaDeg = atof(argv[5]);
    size = atof(argv[6]);

    alpha = alphaDeg * M_PI / 180.0;
    beta = betaDeg  * M_PI / 180.0;

    xB0 = d * cos(beta);
    yB0 = d * sin(beta);

    double v0x = v0 * cos(alpha);
    double v0y = v0 * sin(alpha);

    double tmax = v0y / g;
    double ymax = Y0 + v0y * tmax - 0.5 * g * tmax * tmax;

    double tD = (v0y + sqrt(v0y*v0y + 2*g*Y0)) / g;
    double xmax = v0x * tD;

    FILE *file = fopen("output.txt", "w");

    fprintf(file,
        "y0 = %.3f m; v0 = %.3f m/s; alpha = %.3f deg; a = %.3f m\n",
        Y0, v0, alphaDeg, size);

    fprintf(file,
        "tD = %.3f s; xmax = %.3f m; tmax = %.3f s; ymax = %.3f m\n\n",
        tD, xmax, tmax, ymax);

    fprintf(file,
        "t(s)   xsur(m)   ysur(m)   vx(m/s)   vy(m/s)   v(m/s)\n");

    double tt = 0.0;

    while (tt <= tD)
    {
        double x = v0x * tt;
        double y = Y0 + v0y * tt - 0.5 * g * tt * tt;

        double vx = v0x;
        double vy = v0y - g * tt;
        double v = sqrt(vx*vx + vy*vy);

        fprintf(file,
            "%.3f   %.5f   %.5f   %.5f   %.5f   %.5f\n",
            tt, x, y, vx, vy, v);

        tt += 0.01;
    }

    fclose(file);

    FILE *gnuplot = popen("gnuplot -persistent", "w");

    fprintf(gnuplot, "set grid\n");
    fprintf(gnuplot, "set multiplot layout 3,2 title 'Projectile Motion'\n");

    fprintf(gnuplot, "set xlabel 'Time t [s]'\n");
    fprintf(gnuplot, "set ylabel 'x (m)'\n");
    fprintf(gnuplot, "set yrange [0:%f]\n", xmax * 1.1);
    fprintf(gnuplot, "plot 'output.txt' using 1:2 with lines title 'x(t)'\n");

    fprintf(gnuplot, "set ylabel 'y (m)'\n");
    fprintf(gnuplot, "set yrange [0:%f]\n", ymax * 1.1);
    fprintf(gnuplot, "plot 'output.txt' using 1:3 with lines title 'y(t)'\n");

    fprintf(gnuplot, "set ylabel 'vx (m/s)'\n");
    fprintf(gnuplot, "set yrange [%f:%f]\n", v0x*0.9, v0x*1.1);
    fprintf(gnuplot, "plot 'output.txt' using 1:4 with lines title 'vx(t)'\n");

    fprintf(gnuplot, "set ylabel 'vy (m/s)'\n");
    fprintf(gnuplot, "set yrange [%f:%f]\n", -fabs(v0), fabs(v0));
    fprintf(gnuplot, "plot 'output.txt' using 1:5 with lines title 'vy(t)'\n");

    fprintf(gnuplot, "set ylabel 'v (m/s)'\n");
    fprintf(gnuplot, "set yrange [0:%f]\n", v0 * 1.2);
    fprintf(gnuplot, "plot 'output.txt' using 1:6 with lines title 'v(t)'\n");

    fprintf(gnuplot, "unset multiplot\n");

    fflush(gnuplot);
    pclose(gnuplot);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(900, 600);
    glutCreateWindow("Visualization");

    glClearColor(0.1, 0.3, 0.3, 1.0);

    glutDisplayFunc(drawScene);
    glutReshapeFunc(reshape);
    glutTimerFunc(timeStep, update, 0);

    glutMainLoop();

    return 0;
}