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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <float.h>
#include <math.h>

#include "hull.h"
#include "powercrust.h"

namespace GraspStudio
{

    namespace PowerCrust
    {

        // in powercrust.h
        //extern double est_r;  // guess for r
        //extern double bound[8][3], omins[3], omaxs[3];
        //extern long num_sites;

        /*
        extern struct simplex **pole1, **pole2;
        extern double* lfs_lb; // array of estimated lower bounds on lfs

        extern struct polelabel *adjlist;
        extern struct plist **opplist;
        extern int *rverts;

        extern double  thinthreshold;
        */
        /* variables for tracking infinite loop */
        /* (This should never occur, but it did in early versions) */



        short PowerCrustHull::is_bound(simplex* s)
        {

            int i;

            for (i = 0; i < 4; i++)
                if ((s->neigh[i].vert[0] > pc->omaxs[0]) || (s->neigh[i].vert[0] < pc->omins[0])
                    || (s->neigh[i].vert[1] > pc->omaxs[1]) || (s->neigh[i].vert[1] < pc->omins[1])
                    || (s->neigh[i].vert[2] > pc->omaxs[2]) || (s->neigh[i].vert[2] < pc->omins[2]))
                {
                    return 1;
                }

            return 0;
        }


        void* PowerCrustHull::compute_vv(simplex* s, void* p)
        {
            /* computes Voronoi vertices  */

            static out_func* out_func_here;
            point v[MAXDIM];
            int i, j, k, inf = 0;
            double cc[3], cond, ta[4][3], slvnum, sqrad;

            if (p)
            {
                out_func_here = (out_func*)p;

                if (!s)
                {
                    return NULL;
                }
            }

            for (j = 0; j < cdim; j++)
            {
                v[j] = s->neigh[j].vert;

                /* v[j] stores coordinates of j'th vertex of simplex s; j=0..3 */
                if (v[j] == infinity) /* means simplex s is on the convex hull */
                {
                    inf = 1;
                    break; /* skip the rest of the for loop,
                      ignore convex hull faces (= bounding box ) */
                }

                i = (pc->site_numm)(v[j]); /* i is the index of the vertex v[j] */

                for (k = 0; k < cdim - 1; k++)
                {
                    ta[j][k] = v[j][k] / pc->mult_up; /* restore original coords   */
                    /*    inf=0, ta[0],ta[1],ta[2],ta[3] are 4 vertices of s     */
                }
            }

            if (!inf)   /* if not faces on convex hull, compute circumcenter*/
            {
                tetcircumcenter(ta[0], ta[1], ta[2], ta[3], cc, &cond);
                /* cc is the displacement of circumcenter from ta[0] */
                /* cond is the denominator ( orient3d ) value        */
                sqrad = SQ(cc[0]) + SQ(cc[1]) + SQ(cc[2]);
                slvnum = SQ(cond) / (sqrad * sqrad * sqrad);

                /*    fprintf(DFILE,"%f %f %f\n",cond,slvnum,sqrad);*/
                /*  sqd = 4*maxsqdist(ta[0],ta[1],ta[2],ta[3]); */
                if (cond != 0) /* ignore them if cond = 0 */
                {
                    s->vv = (Coord*) malloc(sizeof(Coord) * 3);

                    for (k = 0; k < cdim - 1; k++)
                    {
                        s->vv[k] = ta[0][k] + cc[k];
                    }

                    /*  if (slvnum<0.00000000001) s->status = PSLV;
                        else */
                    s->status = VV;
                    /*fprintf(CC,"%f %f %f\n",s->vv[0],s->vv[1],s->vv[2]); */
                }
                else   /* if cond=0, s is SLIVER */
                {
                    /*      fprintf(DFILE,"cond=%f sliver!\n", cond); */
                    s->vv = NULL;
                    s->status = SLV;
                    /* modification (no longer used) : set the previous vv as the new vv
                       s->vv = prevsimp->vv;
                       s->status = prevsimp->status; */
                }
            }
            else   /* if on conv hull */
            {
                s->status = CNV;
                /*
                   s->vv = (Coord*) malloc(sizeof(Coord)*3);
                   crossabc(ta[0],ta[1],ta[2],norm);
                   cross product of 3 non-infinite vertices
                   check that this normal is the right sign
                   pointing_in = 0;
                   for (k=0; k<NRAND; k++) {
                   if (dotabc(ta[0],get_site_offline(rverts[k]),norm) > SMALL_ENOUGH)
                   pointing_in = 1;
                   }
                   if (pointing_in) {
                   norm[0] *= -1; norm[1] *= -1; norm[2] *= -1;
                   }
                   s->status = CNV;
                   s->vv[0]=norm[0];
                   s->vv[1]=norm[1];
                   s->vv[2]=norm[2];
                */
            }

            /*      prevsimp = s;  */

            /* computing poles */
            for (j = 0; j < cdim; j++) /* compute 1st pole for vertex j */
            {
                i = (pc->site_numm)(s->neigh[j].vert);

                if (i == -1)
                {
                    continue;
                }

                /* Ignore poles that are too far away to matter - a relic of the
                   original California-style crust. Probably no longer needed */
                if ((s->neigh[j].vert[0] > pc->omaxs[0]) ||
                    (s->neigh[j].vert[0] < pc->omins[0]) ||
                    (s->neigh[j].vert[1] > pc->omaxs[1]) ||
                    (s->neigh[j].vert[1] < pc->omins[1]) ||
                    (s->neigh[j].vert[2] > pc->omaxs[2]) ||
                    (s->neigh[j].vert[2] < pc->omins[2]))
                {

                    /* if (i > (num_sites - 8) ) { /if bounding box vertex */
                    pc->pole1[i] = NULL;
                    continue;
                }

                else
                {

                    if (pc->pole1[i] == NULL)
                    {
                        /* the vertex i is encountered for the 1st time */
                        if (s->status == VV) /* we don't store infinite poles */
                        {
                            pc->pole1[i] = s;
                            continue;
                        }
                    }

                    if ((s->status == VV) && (pc->pole1[i]->status == VV))
                    {
                        if (sqdist(pc->pole1[i]->vv, ta[j]) < sqdist(s->vv, ta[j]))
                        {
                            pc->pole1[i] = s; /* update 1st pole */
                        }
                    }
                }
            }

            return NULL;
        }



        void* PowerCrustHull::compute_pole2(simplex* s, void* p)
        {

            static out_func* out_func_here;
            point v[MAXDIM];
            int i, j, k, inf = 0;
            double a[3];
            site t;
            double dir_s[3], dir_p[3], dist_s, dist_p, cos_sp, est_lfs;
            double cos_2r;

            if (p)
            {
                out_func_here = (out_func*)p;

                if (!s)
                {
                    return NULL;
                }
            }

            for (j = 0; j < cdim; j++)
            {
                v[j] = s->neigh[j].vert;
                i = (pc->site_numm)(v[j]);

                if (i == -1)
                {
                    inf = 1;
                }
            }

            cos_2r = cos(2 * pc->est_r);

            for (j = 0; j < cdim; j++) /* compute 2nd poles */
            {

                t = s->neigh[j].vert;
                i = (pc->site_numm)(t);

                if (i < 0)
                {
                    continue;    /* not a vertex */
                }

                if (inf)   /* on conv hull */
                {
                    if (s->status == CNV)
                    {
                        continue;
                    }
                    else
                    {
                        fprintf(DFILE, "cannot happen7\n");
                    }
                }

                if (!pc->pole1[i])
                {
                    /*        fprintf(DFILE, "no 1st pole\n"); */
                    continue;
                }

                if (pc->pole1[i]->vv == NULL)
                {
                    fprintf(DFILE, "cannot happen8\n");
                    continue;
                }

                if (!s->vv)
                {
                    if (s->status != SLV)
                    {
                        fprintf(DFILE, "cannot happen3\n");
                    }

                    continue;
                }

                for (k = 0; k < cdim - 1; k++) /* a stores orig vertex coord */
                {
                    a[k] = t[k] / pc->mult_up;
                }

                /* compute direction and length of vector from
                   sample to first pole */

                dir_and_dist(a, pc->pole1[i]->vv, dir_p, &dist_p);

                /* We have a vertex, and there is a good first pole. */
                if ((s->status == VV) && (pc->pole1[i]->status == VV))
                {

                    /* make direction vector from sample to this Voronoi vertex */
                    dir_and_dist(a, s->vv, dir_s, &dist_s);

                    /* cosine of angle between angle to vertex and angle to pole */
                    cos_sp = dir_s[0] * dir_p[0] + dir_s[1] * dir_p[1] + dir_s[2] * dir_p[2];

                    /* if there is an estimate for r, use it to estimate lfs */
                    if (pc->est_r < 1.0)
                    {

                        /* near vertices - should be close to sample (a) */
                        if ((cos_sp < cos_2r) && (cos_sp > -cos_2r))
                        {
                            /* use to get lower bound on lfs */
                            est_lfs = dist_s / pc->est_r * ((sqrt(1 - cos_sp * cos_sp)) - pc->est_r);

                            if (est_lfs > pc->lfs_lb[i])
                            {
                                pc->lfs_lb[i] = est_lfs;
                            }
                        }

                    }
                    else
                    {
                        pc->lfs_lb[i] = 0;
                    }

                    if (cos_sp > 0)
                    {
                        /* s->vv is in the same side of pole1  */
                        continue;
                    }

                    /* s->vv is a candidate for pole2 */

                    if (!pc->pole2[i])
                    {
                        /* 1st pole2 candidate for vertex i */

                        pc->pole2[i] = s;
                        continue;
                    }

                    else if (!pc->pole2[i]->vv)   /* 2nd pole points null */
                    {
                        fprintf(DFILE, "cannot happen4\n");
                        continue;
                    }

                    else if ((pc->pole2[i]->status == VV) &&
                             (sqdist(a, pc->pole2[i]->vv) < sqdist(a, s->vv)))
                    {
                        pc->pole2[i] = s;    /* update 2nd pole */
                    }

                }
            }

            /*  out_func_here(v,cdim,DFILE,0); */

            return NULL;
        }

        /* tests pole to see if it's farther than estimated local feature size.
           v is a sample, p is a pole. */

        int PowerCrustHull::close_pole(double* v, double* p, double lfs_lb)
        {
            return (sqdist(v, p) < lfs_lb * lfs_lb);
        }


        int PowerCrustHull::antiLabel(int label)
        {
            if (label == HULL_IN)
            {
                return (HULL_OUT);
            }

            if (label == HULL_OUT)
            {
                return (HULL_IN);
            }

            return (label);
        }

        /* labels a pole */
        void PowerCrustHull::labelPole(int pid, int label)
        {
            pc->adjlist[pid].label = label;

            if (pid == crust_loopStart)
            {
                crust_loopStart = -1;
            }
        }

        /* checks to see if list of unlabeled poles is looping infinitely */
        int PowerCrustHull::cantLabelAnything(int pid)
        {

            if (crust_loopStart == -1)
            {
                crust_loopStart = pid;
                crust_count = 0;
                crust_lastCount = 0;
                return (crust_FALSE_i);
            }

            if (pid == crust_loopStart)
            {
                if (crust_count == crust_lastCount)
                {
                    /* infinite loop! */
                    fprintf(DFILE, "Can't label any more! \n");
                    return (crust_TRUE_i);
                }
                else
                {
                    /* we labeled something last time through */
                    crust_lastCount = crust_count;
                    crust_count = 0;
                }
            }
            else
            {
                /* in the middle of the loop */
                crust_count++;
            }

            return (crust_FALSE_i);
        }


        /* computes angle between two opposite poles */
        double PowerCrustHull::computePoleAngle(simplex* pole1, simplex* pole2, double* samp)
        {

            return (((pole1->vv[0] - samp[0]) * (pole2->vv[0] - samp[0]) +
                     (pole1->vv[1] - samp[1]) * (pole2->vv[1] - samp[1]) +
                     (pole1->vv[2] - samp[2]) * (pole2->vv[2] - samp[2])) /
                    (sqrt(SQ(pole1->vv[0] - samp[0]) +
                          SQ(pole1->vv[1] - samp[1]) +
                          SQ(pole1->vv[2] - samp[2])) *
                     sqrt(SQ(pole2->vv[0] - samp[0]) +
                          SQ(pole2->vv[1] - samp[1]) +
                          SQ(pole2->vv[2] - samp[2]))));
        }

        /* Adds a new pair of opposite poles to each other's lists */
        void PowerCrustHull::newOpposite(int p1index, int p2index, double pole_angle)
        {
            plist*  newplist;

            newplist = (struct plist*) malloc(sizeof(struct plist));
            newplist->pid = p2index;
            newplist->angle = pole_angle;
            newplist->next = pc->opplist[p1index];
            pc->opplist[p1index] = newplist;

            if (pc->adjlist[p1index].oppradius > pc->adjlist[p2index].sqradius)
            {
                assert(pc->adjlist[p2index].sqradius > 0.0);
                pc->adjlist[p1index].oppradius = pc->adjlist[p2index].sqradius;
            }
        }


        /* Outputs a pole, saving it's squared radius in adjlist */
        void PowerCrustHull::outputPole(FILE* POLE, FILE* SPFILE, simplex* pole, int poleid,
                                        double* samp, int* num_poles, double distance)
        {
            double r2, weight;


            //=== MP 2013 Modifications: Store surface points for each sphere!

            int j2, k2;
            point v[MAXDIM];

            /* get the coordinates of the  four endpoints */
            for (j2 = 0; j2 < 4; j2++)
            {
                v[j2] = pole->neigh[j2].vert;

                for (k2 = 0; k2 < 3; k2++)
                {
                    pc->adjlist[poleid].surface_points[j2][k2] = v[j2][k2] / pc->mult_up;
                }

                //indices[j2][k2]=v[j2][k2]/pc->mult_up;
            }

            //===

            r2 = SQ(pole->vv[0] - samp[0])
                 + SQ(pole->vv[1] - samp[1])
                 + SQ(pole->vv[2] - samp[2]);

            weight = SQ(pole->vv[0]) + SQ(pole->vv[1]) + SQ(pole->vv[2]) - r2;

            pole->status = POLE_OUTPUT;
            pole->poleindex = poleid;

            /* debugging file */
            fprintf(POLE, "%f %f %f\n", pole->vv[0],
                    pole->vv[1], pole->vv[2]);

            /* for computing powercrust */
            fprintf(SPFILE, "%f %f %f %f\n", pole->vv[0],
                    pole->vv[1], pole->vv[2],
                    weight);


            /* remember squared radius */
            pc->adjlist[poleid].sqradius = r2;
            pc->adjlist[poleid].samp_distance = distance;

            /* initialize perp dist to MA */
            pc->adjlist[poleid].oppradius = r2;

            /* initialize */
            pc->adjlist[poleid].grafindex = -1;

            /* keep count! */
            (*num_poles)++;

        }

    }
}
