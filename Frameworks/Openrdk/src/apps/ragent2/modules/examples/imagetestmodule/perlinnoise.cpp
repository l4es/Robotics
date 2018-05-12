#include <cmath>
using namespace std;

#include "perlinnoise.h"

double noise1(int x)
{
   x = (x<<13) ^ x;
   return (double)( 1.0 - ( (x * (x * x * 15731 + 789221)
                           + 1376312589) & 0x7fffffff ) / 1073741824.0);
}

double noise2(int x, int y)
{
   int n;
   n = x + y * 57;
   return noise1(n);
}

double noise3(int x, int y, int z)
{
    return noise1(x + y * 57 + z * 57 * 57);
}

double smoothnoise1(int x)
{
    return noise1(x) / 2 + noise1(x-1) / 4 + noise1(x+1) / 4;
}

double smoothnoise2(int x, int y)
{
    double corners = ( noise2(x-1, y-1)+noise2(x+1, y-1)+noise2(x-1, y+1)+noise2(x+1, y+1) ) / 16;
    double sides = ( noise2(x-1, y)  +noise2(x+1, y)  +noise2(x, y-1)  +noise2(x, y+1) ) /  8;
    double center = noise2(x, y) / 4;

    return corners + sides + center;
}

double smoothnoise3(int x, int y, int z)
{
    double a = 0;
    for (int xx = x-1; xx <= x+1; xx++)
        for (int yy = y-1; yy <= y+1; yy++)
            for (int zz = z-1; zz <= z+1; zz++)
                if (xx != x || yy != y || zz != z) a += noise3(xx,yy,zz);

    a /= 32;
    a += noise3(x,y,z) / 16;
    return a;
}

double interp(double x1, double x2, double a)
{
    double b = (1-a);
    double a2 = a*a, a3 = a2*a;
    double b2 = b*b, b3 = b2*b;
    return x1 * (3 * b2 - 2 * b3) + x2 * (3 * a2 - 2 * a3);
}

double perlinNoise1D(double x, double persistence, bool smooth,
    unsigned int octaves)
{
    double res = 0;
    double freq = 1;
    double ampl = 1;

    for (unsigned int i = 0; i < octaves; i++) {
        double xFreq = x * freq;
        int xInt = (int) xFreq;
        double xFrac = xFreq - xInt;
        double left = smooth ? smoothnoise1(xInt) : noise1(xInt);
        double right = smooth ? smoothnoise1(xInt+1) : noise1(xInt+1);
        double v = interp(left, right, xFrac);
        res += v * ampl;
        freq *= 2;
        ampl *= persistence;
    }
    return res;
}

double perlinNoise2D(double x, double y, double persistence,
    bool smooth, unsigned int octaves)
{
    double res = 0;
    double freq = 1;
    double ampl = 1;

    for (unsigned int i = 0; i < octaves; i++) {
        double xFreq = x * freq, yFreq = y * freq;
        int xInt = (int) xFreq;
        int yInt = (int) yFreq;
        double xFrac = xFreq - xInt;
        double yFrac = yFreq - yInt;
        double topleft = smooth ? smoothnoise2(xInt, yInt) : noise2(xInt, yInt);
        double topright = smooth ? smoothnoise2(xInt+1, yInt) : noise2(xInt+1, yInt);
        double bottomleft = smooth ? smoothnoise2(xInt, yInt+1) : noise2(xInt, yInt+1);
        double bottomright = smooth ? smoothnoise2(xInt+1, yInt+1) : noise2(xInt+1, yInt+1);
        double v = interp(
            interp(topleft, topright, xFrac),
            interp(bottomleft, bottomright, xFrac),
            yFrac);
        res += v * ampl;
        freq *= 2;
        ampl *= persistence;
    }
    if (res > 0.5) res = 0.5;
    if (res < -0.5) res = -0.5;
    return res;
}

double perlinNoise3D(double x, double y, double z, double persistence,
    bool smooth, unsigned int octaves)
{
    double res = 0;
    double freq = 1;
    double ampl = 1;
    double finalscale = 0;

    for (unsigned int i = 0; i < octaves; i++) {
        double xFreq = x * freq, yFreq = y * freq, zFreq = z * freq;
        int xInt = (int) xFreq;
        int yInt = (int) yFreq;
        int zInt = (int) zFreq;
        double xFrac = xFreq - xInt;
        double yFrac = yFreq - yInt;
        double zFrac = zFreq - zInt;
        double topleft = smooth ? smoothnoise3(xInt, yInt, zInt) : noise3(xInt, yInt, zInt);
        double topright = smooth ? smoothnoise3(xInt+1, yInt, zInt) : noise3(xInt+1, yInt, zInt);
        double bottomleft = smooth ? smoothnoise3(xInt, yInt+1, zInt) : noise3(xInt, yInt+1, zInt);
        double bottomright = smooth ? smoothnoise3(xInt+1, yInt+1, zInt) : noise3(xInt+1, yInt+1, zInt);
        double topleftz = smooth ? smoothnoise3(xInt, yInt, zInt+1) : noise3(xInt, yInt, zInt+1);
        double toprightz = smooth ? smoothnoise3(xInt+1, yInt, zInt+1) : noise3(xInt+1, yInt, zInt+1);
        double bottomleftz = smooth ? smoothnoise3(xInt, yInt+1, zInt+1) : noise3(xInt, yInt+1, zInt+1);
        double bottomrightz = smooth ? smoothnoise3(xInt+1, yInt+1, zInt+1) : noise3(xInt+1, yInt+1, zInt+1);
        double v = interp(
            interp(topleft, topright, xFrac),
            interp(bottomleft, bottomright, xFrac),
            yFrac);
        double vz = interp(
            interp(topleftz, toprightz, xFrac),
            interp(bottomleftz, bottomrightz, xFrac),
            yFrac);
        double vtrue = interp(v, vz, zFrac);
        res += vtrue * ampl;
        finalscale += ampl;
        freq *= 2;
        ampl *= persistence;
    }
    res /= finalscale;
    if (res > 0.5) res = 0.5;
    if (res < -0.5) res = -0.5;
    return res;
}

/* Coherent noise function over 1, 2 or 3 dimensions */
/* (copyright Ken Perlin) */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define B 0x100
#define BM 0xff
#define N 0x1000
#define NP 12   /* 2^N */
#define NM 0xfff

#define s_curve(t) ( t * t * (3. - 2. * t) )
#define lerp(t, a, b) ( a + t * (b - a) )
#define setup(i,b0,b1,r0,r1)\
        t = vec[i] + N;\
        b0 = ((int)t) & BM;\
        b1 = (b0+1) & BM;\
        r0 = t - (int)t;\
        r1 = r0 - 1.;
#define at2(rx,ry) ( rx * q[0] + ry * q[1] )
#define at3(rx,ry,rz) ( rx * q[0] + ry * q[1] + rz * q[2] )

void init(void);
double orignoise1(double);
double orignoise2(double *);
double orignoise3(double *);
void orignormalize3(double *);
void orignormalize2(double *);

static int p[B + B + 2];
static double g3[B + B + 2][3];
static double g2[B + B + 2][2];
static double g1[B + B + 2];
static int start = 1;

double orignoise1(double arg)
{
   int bx0, bx1;
   double rx0, rx1, sx, t, u, v, vec[1];

   vec[0] = arg;
   if (start) {
      start = 0;
      init();
   }

   setup(0,bx0,bx1,rx0,rx1);

   sx = s_curve(rx0);
   u = rx0 * g1[ p[ bx0 ] ];
   v = rx1 * g1[ p[ bx1 ] ];

   return(lerp(sx, u, v));
}

double orignoise2(double vec[2])
{
   int bx0, bx1, by0, by1, b00, b10, b01, b11;
   double rx0, rx1, ry0, ry1, *q, sx, sy, a, b, t, u, v;
   int i, j;

   if (start) {
      start = 0;
      init();
   }

   setup(0, bx0,bx1, rx0,rx1);
   setup(1, by0,by1, ry0,ry1);

   i = p[ bx0 ];
   j = p[ bx1 ];

   b00 = p[ i + by0 ];
   b10 = p[ j + by0 ];
   b01 = p[ i + by1 ];
   b11 = p[ j + by1 ];

   sx = s_curve(rx0);
   sy = s_curve(ry0);

   q = g2[ b00 ] ; u = at2(rx0,ry0);
   q = g2[ b10 ] ; v = at2(rx1,ry0);
   a = lerp(sx, u, v);

   q = g2[ b01 ] ; u = at2(rx0,ry1);
   q = g2[ b11 ] ; v = at2(rx1,ry1);
   b = lerp(sx, u, v);

   return lerp(sy, a, b);
}

double orignoise3(double vec[3])
{
   int bx0, bx1, by0, by1, bz0, bz1, b00, b10, b01, b11;
   double rx0, rx1, ry0, ry1, rz0, rz1, *q, sy, sz, a, b, c, d, t, u, v;
   int i, j;

   if (start) {
      start = 0;
      init();
   }

   setup(0, bx0,bx1, rx0,rx1);
   setup(1, by0,by1, ry0,ry1);
   setup(2, bz0,bz1, rz0,rz1);

   i = p[ bx0 ];
   j = p[ bx1 ];

   b00 = p[ i + by0 ];
   b10 = p[ j + by0 ];
   b01 = p[ i + by1 ];
   b11 = p[ j + by1 ];

   t  = s_curve(rx0);
   sy = s_curve(ry0);
   sz = s_curve(rz0);

   q = g3[ b00 + bz0 ] ; u = at3(rx0,ry0,rz0);
   q = g3[ b10 + bz0 ] ; v = at3(rx1,ry0,rz0);
   a = lerp(t, u, v);

   q = g3[ b01 + bz0 ] ; u = at3(rx0,ry1,rz0);
   q = g3[ b11 + bz0 ] ; v = at3(rx1,ry1,rz0);
   b = lerp(t, u, v);

   c = lerp(sy, a, b);

   q = g3[ b00 + bz1 ] ; u = at3(rx0,ry0,rz1);
   q = g3[ b10 + bz1 ] ; v = at3(rx1,ry0,rz1);
   a = lerp(t, u, v);

   q = g3[ b01 + bz1 ] ; u = at3(rx0,ry1,rz1);
   q = g3[ b11 + bz1 ] ; v = at3(rx1,ry1,rz1);
   b = lerp(t, u, v);

   d = lerp(sy, a, b);

   return lerp(sz, c, d);
}

void orignormalize2(double v[2])
{
   double s;

   s = sqrt(v[0] * v[0] + v[1] * v[1]);
   v[0] = v[0] / s;
   v[1] = v[1] / s;
}

void orignormalize3(double v[3])
{
   double s;

   s = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
   v[0] = v[0] / s;
   v[1] = v[1] / s;
   v[2] = v[2] / s;
}

void init(void)
{
   int i, j, k;

   for (i = 0 ; i < B ; i++) {
      p[i] = i;
      g1[i] = (double)((rand() % (B + B)) - B) / B;

      for (j = 0 ; j < 2 ; j++)
         g2[i][j] = (double)((rand() % (B + B)) - B) / B;
      orignormalize2(g2[i]);

      for (j = 0 ; j < 3 ; j++)
         g3[i][j] = (double)((rand() % (B + B)) - B) / B;
      orignormalize3(g3[i]);
   }

   while (--i) {
      k = p[i];
      p[i] = p[j = rand() % B];
      p[j] = k;
   }

   for (i = 0 ; i < B + 2 ; i++) {
      p[B + i] = p[i];
      g1[B + i] = g1[i];
      for (j = 0 ; j < 2 ; j++)
         g2[B + i][j] = g2[i][j];
      for (j = 0 ; j < 3 ; j++)
         g3[B + i][j] = g3[i][j];
   }
}

/* --- My harmonic summing functions - PDB --------------------------*/

/*
   In what follows "alpha" (1 / persistence) is the weight when the sum is formed.
   Typically it is 2, As this approaches 1 the function is noisier.
   "beta" is the harmonic scaling/spacing, typically 2.
*/

double originalPerlinNoise1D(double x,double persistence,double beta,int octaves)
{
   int i;
   double alpha = 1 / persistence;
   double val,sum = 0;
   double p,scale = 1;
   double finalscale = 0;

   p = x;
   for (i=0;i<octaves;i++) {
      val = orignoise1(p);
      sum += val / scale;
      finalscale += 1. / scale;
      scale *= alpha;
      p *= beta;
   }
   sum /= finalscale;
   if (sum > 0.5) sum = 0.5;
   if (sum < -0.5) sum = -0.5;
   return(sum);
}

double originalPerlinNoise2D(double x,double y,double persistence,double beta,int octaves)
{
   int i;
   double alpha = 1 / persistence;
   double val,sum = 0;
   double p[2],scale = 1;
   double finalscale = 0;

   p[0] = x;
   p[1] = y;
   for (i=0;i<octaves;i++) {
      val = orignoise2(p);
      sum += val / scale;
      finalscale += 1. / scale;
      scale *= alpha;
      p[0] *= beta;
      p[1] *= beta;
   }
   sum /= finalscale;
   if (sum > 0.5) sum = 0.5;
   if (sum < -0.5) sum = -0.5;
   return(sum);
}

double originalPerlinNoise3D(double x,double y,double z,double persistence,double beta,int octaves)
{
   int i;
   double alpha = 1 / persistence;
   double val,sum = 0;
   double p[3],scale = 1;
   double finalscale = 0;

   p[0] = x;
   p[1] = y;
   p[2] = z;
   for (i=0;i<octaves;i++) {
      val = orignoise3(p);
      sum += val / scale;
      finalscale += 1. / scale;
      scale *= alpha;
      p[0] *= beta;
      p[1] *= beta;
      p[2] *= beta;
   }
   sum /= finalscale;
   if (sum > 0.5) sum = 0.5;
   if (sum < -0.5) sum = -0.5;
   return(sum);
}

