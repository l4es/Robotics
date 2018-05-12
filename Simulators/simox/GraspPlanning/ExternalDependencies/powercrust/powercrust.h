/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

/*
 * This code is based on the PowerCrust hullmain example.
 *
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

/*
 * This file is a significant modification of Ken Clarkson's file hullmain.c.
 * We include his copyright notice in accordance with its terms.
 *                                                                     - Nina, Sunghee and Ravi
 */


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

#ifndef _GraspStudio_PowerCrust_h_
#define _GraspStudio_PowerCrust_h_

#include "GraspPlanning/GraspStudio.h"
#include <float.h>
//#include <math.h>
#include <cmath>    //MP
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <string.h>
#include <ctype.h>

#include <string>

#include "hull.h"
#include "pc_getopt.h"
#include "pointsites.h"



//#define POINTSITES 1


namespace GraspStudio
{
    namespace PowerCrust
    {

        class PowerCrustHull;

        class PowerCrust: public boost::enable_shared_from_this<GraspStudio::PowerCrust::PowerCrust>
        {
            friend class PowerCrustHull;
        public:
            /*!
                Constructor
            */
            PowerCrust();

            bool init(std::vector<Eigen::Vector3f>& vertices);

            bool computeMedialAxis();

            //MP 2013-08-27
            //bypass file i/o
            std::vector<PolarBall> resultingPolarBalls;

            void setVerbose(bool enable);

        protected:
            void init();
            site new_site(site p, long j);
            site buildSite(long j, const Eigen::Vector3f& v);
            bool buildBoundingBox(long j);
            site get_site_offline(long i);
            void make_shuffle(void);
            void make_shuffle_repaired_MP(void);
            long mat_size;   //MP replacing a static local variable in make_shuffle() that caused problems.

            long noshuffle(long i);
            long shufflef(long i);
            //long (*shuf)(long); // the shuffle function
            site get_next_site(void);

            site read_next_site(long j);

            //site (*get_site_n)(long);
            long* shufmat;
            std::vector<Eigen::Vector3f> vertices;
            int dim; // the input vertex dimension
            double mult_up;
            Coord mins[MAXDIM], maxs[MAXDIM]; // min max values (mult_up applied)
            double est_r;  /* guess for r */
            bool generateOutput; // write output to file
            double bound[8][3], omaxs[3], omins[3];  /* 8 vertices for bounding box */
            point   site_blocks[MAXBLOCKS];
            int site_size, /* size of malloc needed for a site */
                point_size;  /* size of malloc needed for a point */

            long seed; // rand seed

            int num_blocks;
            int power_numfaces;
            struct queuenode* queue;
            struct queuenode* qend;
            int num_vtxs, num_faces;

            site p;          /* the current site */

            long site_numm(site p);


            /* Data structures for poles */
            struct simplex** pole1, ** pole2; /* arrays of poles - per sample*/
            struct polelabel* adjlist;  /* array of additional info - per pole */
            struct plist** opplist; /* opposite pid and angle between poles - per pole */
            double* lfs_lb;  /*  array of lower bounds for lfs of each sample */


            int num_poles, num_axedgs, num_axfaces;


            double* pole1_distance, *pole2_distance;


            /* for priority queue */
            //int heap_size;

            /* int  getopt(int, char**, char*); */
            char* optarg;
            int optind;
            int opterr;
            int scount;

            long num_sites;
            short vd;

            short power_diagram; /* 1 if power diagram */

            long s_num; /* site number */

            double theta; /* input argument - angle defining deep intersection */
            double deep; /* input argument.. same as theta for labeling unlabled pole */
            int defer; /* input argument -D 1 if you don't want to propagate bad poles */

            int poleInput; /* are the poles given as input */

            FILE*  INFILE, *OUTFILE, *DFILE, *TFILE, *SPFILE, *POLE, *PC, *PNF, *INPOLE, *INPBALL, *INVBALL, *AXIS, *AXISFACE, *OUTPOLE, *POLEINFO, *HEAD;

            int* rverts;
            double samp[3];
            int numbadpoles;
            long poleid;
            short bad;
            double  pole_angle;
            simplex* root;
            boost::shared_ptr<PowerCrustHull> hull;
            out_func mof;
            visit_func pr;

            long(PowerCrust::*shuf)(long);
            site(PowerCrust::*get_site_n)(long);
            void make_output(simplex* root, void* (PowerCrustHull::*visit_gen)(simplex*, visit_func visit), visit_func visit, out_func out_funcp, FILE* F, boost::shared_ptr<PowerCrustHull> hull);


            // step 1
            bool buildConvexHull();
            // step 2
            bool buildPoles1();
            // step 3
            bool buildPoles2();
            // step 4
            bool buildPowerVV();
            // step 5
            bool buildLabels();
            // step 6
            bool buildCrust();
            // step 7
            bool buildMedialAxis();
        };

        typedef boost::shared_ptr<PowerCrust> PowerCrustPtr;

    } // namespace

} // namespace

#endif //_GraspStudio_PowerCrust_h_
