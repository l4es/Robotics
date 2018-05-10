/*
 * Ken Clarkson wrote this.  Copyright (c) 1995 by AT&T..
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY.  IN PARTICULAR, NEITHER THE AUTHORS NOR AT&T MAKE ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdio.h>
#include <time.h>
#include "hull.h"
#include <limits>

//double erand48 (unsigned short X[3]);
#include "randStuff.h"


namespace GraspStudio
{

    namespace PowerCrust
    {

#define RAND48_SEED_0   (0x330e)
#define RAND48_SEED_1   (0xabcd)
#define RAND48_SEED_2   (0x1234)
#define RAND48_MULT_0   (0xe66d)
#define RAND48_MULT_1   (0xdeec)
#define RAND48_MULT_2   (0x0005)
#define RAND48_ADD  (0x000b)

        unsigned short __rand48_Seed[3] =
        {
            RAND48_SEED_0,
            RAND48_SEED_1,
            RAND48_SEED_2
        };

        unsigned short __rand48_Mult[3] =
        {
            RAND48_MULT_0,
            RAND48_MULT_1,
            RAND48_MULT_2
        };

        unsigned short __rand48_Add = RAND48_ADD;


        unsigned short X[3];

        static void _dorand48(unsigned short xseed[3])
        {
            unsigned long accu;
            unsigned short temp[2];

            accu = (unsigned long) __rand48_Mult[0] * (unsigned long) xseed[0] + (unsigned long) __rand48_Add;
            temp[0] = (unsigned short) accu;    /* lower 16 bits */
            accu >>= sizeof(unsigned short) * 8;
            accu += (unsigned long) __rand48_Mult[0] * (unsigned long) xseed[1] + (unsigned long) __rand48_Mult[1] * (unsigned long) xseed[0];
            temp[1] = (unsigned short) accu;    /* middle 16 bits */
            accu >>= sizeof(unsigned short) * 8;
            accu += __rand48_Mult[0] * xseed[2] + __rand48_Mult[1] * xseed[1] + __rand48_Mult[2] * xseed[0];
            xseed[0] = temp[0];
            xseed[1] = temp[1];
            xseed[2] = (unsigned short) accu;
        }

        double  erand48(unsigned short xseed[3])
        {
            _dorand48(xseed);
            return ldexp((double) xseed[0], -48) + ldexp((double) xseed[1], -32) + ldexp((double) xseed[2], -16);
        }

        double double_rand(void)
        {
            return erand48(X);
        }

        void init_rand(long seed)
        {
            fprintf(stderr, "init_rand: seed = %d\n",
                    X[1] = (seed == 0) ? (int)time(0) : (int)seed);
        }


        //#ifdef cray
        double logb(double x)
        {
            if (x <= 0)
            {
                return std::numeric_limits<double>::min();    //-1e2460;
            }

            return log((double)x) / log((double)2);
        }
        //#endif
    }

}
