#ifndef PerlinNoiseH
#define PerlinNoiseH

double perlinNoise1D(double x,                     double persistence, bool smooth = true, unsigned int octaves = 5);
double perlinNoise2D(double x, double y,           double persistence, bool smooth = true, unsigned int octaves = 5);
double perlinNoise3D(double x, double y, double z, double persistence, bool smooth = true, unsigned int octaves = 5);
double originalPerlinNoise1D(double x,                     double persistence, double beta, int octaves = 5);
double originalPerlinNoise2D(double x, double y,           double persistence, double beta, int octaves = 5);
double originalPerlinNoise3D(double x, double y, double z, double persistence, double beta, int octaves = 5);

#endif
