/*
 * Power Crust software, by Nina Amenta, Sunghee Choi and Ravi Krishna Kolluri.
 * Copyright (c) 2000 by the University of Texas
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee under the GNU Public License is hereby granted,
 * provided that this entire notice  is included in all copies of any software
 * which is or includes a copy or modification of this software and in all copies
 * of the supporting documentation for such software.
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY.  IN PARTICULAR, NEITHER THE AUTHORS NOR AT&T MAKE ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 */

#ifndef PowerCrust_PointSites
#define PowerCrust_PointSites

#include "points.h"

namespace GraspStudio
{

    namespace PowerCrust
    {

        const int MAXDIM = 8;
        const int BLOCKSIZE = 100000;
        const int MAXBLOCKS = 1000;
        const int DEBUG = -7;
        const int CHECK_OVERSHOOT = 1;
        const int EXACT = 1; /* sunghee */
        const int NRAND = 5; /* number of random points chosen to check orientation */
        const double SMALL_ENOUGH = .0001;
        const int MAXNF = 100; /* max number of conv hull triangles adjacent to a vertex */
        const int MAXTA = 100000;
        const int MAXTEMPA = 100000;
        const int CNV = 0; /* sunghee : status of simplex, if it's on convex hull */
        const int VV = 1; /* sunghee :    if it's regular simplex  */
        const int SLV = -1; /* sunghee : if orient3d=0, sliver simplex */
        const int AV = 2; /* if av contains the averaged pole vector */
        const int PSLV = -2; /* probably sliver */
        const int POLE_OUTPUT = 3; /* VV is pole and it's ouput */
        //double SQ(double a) {return a*a;} /* sunghee */

        const int BAD_POLE = -1;

        const int HULL_IN = 2;
        const int HULL_OUT = 1;
        const int HULL_INIT = 0;
        const int HULL_NONE = -1;

        const int FIRST = 0;
        const int NO = 1;
        const int DEG = -1;
        const int NORM = 2;
        const int VOR = 3;
        const int VOR_NORM = 4;
        const int SLVT = 7;
        const int PSLVT = 8;
        const int SURF = 5;
        const int OPP = 6;

        const int FIRST_EDGE = 0;
        const int POW = 1;
        const int NOT_POW = 2;
        const int VISITED = 3;

        /*RAVI */

        const int VALIDEDGE = 24;
        const int INVALIDEDGE = 23;
        const int INEDGE = 25;
        const int OUTEDGE = 26;
        const int ADDAXIS = 13;
        const int PRESENT = 19;
        const int FIXED = 20;
        const int REMOVED = 21; /* for the thinning  stuff */

        /* for priority queue */
        //int LEFT(int i){return (i)*2;}
        //int RIGHT(int i){return (i)*2+1;}
        //int PARENT(int i){return (i)/2;}

        //const int MAXBLOCKS=10000;

        typedef point site;
        //typedef Coord* normalp;
        //point site_blocks[MAXBLOCKS];
        //int   num_blocks;

    }
}

#endif
