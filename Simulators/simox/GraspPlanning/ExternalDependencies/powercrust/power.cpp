#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <float.h>
#include <math.h>

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

#include "hull.h"
#include "powercrust.h"

#ifdef WIN32
#pragma warning(push)
#pragma warning(disable: 4996) //4996 for _CRT_SECURE_NO_WARNINGS equivalent
#endif


namespace GraspStudio
{

    namespace PowerCrust
    {

        //extern double theta;
        //extern FILE *PC, *PNF, *INFPOLE,*AXIS,*AXISFACE; /* *RT, *PS, *PR; */

        //extern int num_vtxs,num_faces;

        //extern double theta;
        //extern struct polelabel *adjlist;
        //extern struct queuenode *queue;
        //extern long site_numm(site p);
        //extern void triorthocenter(double a[], double b[], double c[],
        //                           double orthocenter[], double* cnum);
        //extern void tetorthocenter(double a[], double b[], double c[], double d[],
        //                          double orthocenter[], double* cnum);

        /* some new variables */

        //extern int num_poles,num_axedgs,num_axfaces;



        void* PowerCrustHull::compute_2d_power_vv(simplex* s, void* p)
        {
            /* computes Voronoi vertices  */

            static out_func* out_func_here;
            point v[MAXDIM];
            int j, k, inf = 0, index;
            double cc[2], cond, ta[3][3];
            cond = 1.0f; // must be !=0 in !EXACT mode

            if (p)
            {
                out_func_here = (out_func*)p;

                if (!s)
                {
                    return NULL;
                }
            }

            index = 0;

            for (j = 0; j < 3; j++)
            {
                v[j] = s->neigh[j].vert;

                /* v[j] stores coordinates of j'th vertex of simplex s; j=0..3 */
                if (v[j] == infinity) /* means simplex s is on the convex hull */
                {
                    inf = 1;
                    continue; /* skip the rest of the for loop; process next vertex */
                }

                /*i=(site_num)(v[j]); i is the index of the vertex v[j] */
                for (k = 0; k < 3; k++)
                {
                    ta[index][k] = v[j][k] / pc->mult_up;
                    /* restore original coords   */
                    /* if inf=1, ta[0],ta[1] are non-infinite vertices of s*/
                    /*    inf=0, ta[0],ta[1],ta[2] are 3 vertices of s     */
                }

                index++;
            }

            printf("\n");

            if (!inf)   /* if not faces on convex hull, compute circumcenter*/
            {
                for (k = 0; k < 3; k++)
                    /*    printf("%f %f %f\n",ta[k][0],ta[k][1],ta[k][2]);*/
                {
                    triorthocenter(ta[0], ta[1], ta[2], cc, &cond);
                }

                /* cc is the displacement of orthocenter from ta[0] */
                /* cond is the denominator ( orient2d ) value        */
                if (cond != 0) /* ignore them if cond = 0 */
                {
                    s->vv = (Coord*) malloc(sizeof(Coord) * 2);

                    for (k = 0; k < 2; k++)
                    {
                        s->vv[k] = ta[0][k] + cc[k];
                    }

                    s->status = VV;

                }
                else   /* if cond=0, s is SLIVER */
                {
                    fprintf(DFILE, "sliver!\n");
                    s->vv = NULL;
                    s->status = SLV;
                }
            }
            else   /* if on conv hull, ignore */
            {
                s->vv = NULL;
                s->status = CNV;
            }

            return NULL;
        }

        /*
        void *reg_triang(simplex *s, void *p) {

            static out_func *out_func_here;
            point v[MAXDIM];
            int j,k,vnum;
                double cc[3], cond, ta[4][4];

            if (p) {out_func_here = (out_func*)p; if (!s) return NULL;}

            for (j=0;j<cdim;j++) {
              v[j] = s->neigh[j].vert;

              if (v[j]==infinity) {
                return NULL;
              }
            }

            for (j=0;j<cdim;j++) {
                        vnum=0;
                        for (k=0;k<cdim;k++) {
                                if (k==j) continue;
                                v[vnum++] = (s->neigh[k].vert);
                        }
                    fprintf(RT,"3 %d %d %d\n",(site_num)(v[0]),(site_num)(v[1]),(site_num)(v[2]));
                }
                return NULL;
        }
        */


        void* PowerCrustHull::compute_3d_power_vv(simplex* s, void* p)
        {

            static out_func* out_func_here;
            point v[MAXDIM];
            int j, k, inf = 0, index, visited_edge;
            double cc[3], cond, ta[4][4], d, r1, r2, e;
            struct edgesimp* newplist, *pindex;

            if (p)
            {
                out_func_here = (out_func*)p;

                if (!s)
                {
                    return NULL;
                }
            }

            index = 0;

            for (j = 0; j < cdim; j++)
            {
                v[j] = s->neigh[j].vert;

                /* v[j] stores coordinates of j'th vertex of simplex s; j=0..3 */
                if (v[j] == infinity) /* means simplex s is on the convex hull */
                {
                    inf = 1;
                    continue; /* skip the rest of the for loop; process next vertex */
                }

                /*i=(site_num)(v[j]);  i is the index of the vertex v[j] */
                for (k = 0; k < 4; k++)
                {
                    ta[index][k] = v[j][k] / pc->mult_up; /* restore original coords   */
                    /* if inf=1, ta[0],ta[1],ta[2] are non-infinite vertices of s*/
                    /*    inf=0, ta[0],ta[1],ta[2],ta[3] are 4 vertices of s     */
                }

                index++;
            }

            /* if not faces on convex hull, process */
            if (!inf)
            {

                /* build structure for each edge, including angle of intersection */
                for (k = 0; k < 6; k++)
                {
                    if (s->edgestatus[k] == FIRST_EDGE) /* not visited edge */
                    {
                        pindex = pc->adjlist[pc->site_numm(v[power_v1[k]])].eptr;
                        visited_edge = 0;

                        while (pindex != NULL)
                        {
                            if (pindex->pid == pc->site_numm(v[power_v2[k]]))   /* already in the list */
                            {
                                visited_edge = 1;
                                break;
                            }

                            pindex = pindex->next;
                        }

                        if (!visited_edge)
                        {
                            d = sqdist(ta[power_v1[k]], ta[power_v2[k]]);
                            r1 = SQ(ta[power_v1[k]][0]) + SQ(ta[power_v1[k]][1]) + SQ(ta[power_v1[k]][2]) - ta[power_v1[k]][3];
                            r2 = SQ(ta[power_v2[k]][0]) + SQ(ta[power_v2[k]][1]) + SQ(ta[power_v2[k]][2]) - ta[power_v2[k]][3];
                            e = 2 * sqrt(r1) * sqrt(r2);

                            newplist = (struct edgesimp*) malloc(sizeof(struct edgesimp));
                            newplist->simp = s;
                            newplist->kth = k;
                            newplist->angle = (r1 + r2 - d) / e;
                            newplist->pid = pc->site_numm(v[power_v1[k]]);
                            newplist->next = pc->adjlist[pc->site_numm(v[power_v2[k]])].eptr;
                            pc->adjlist[pc->site_numm(v[power_v2[k]])].eptr = newplist;

                            newplist = (struct edgesimp*) malloc(sizeof(struct edgesimp));
                            newplist->simp = s;
                            newplist->kth = k;
                            newplist->angle = (r1 + r2 - d) / e;
                            newplist->pid = pc->site_numm(v[power_v2[k]]);
                            newplist->next = pc->adjlist[pc->site_numm(v[power_v1[k]])].eptr;
                            pc->adjlist[pc->site_numm(v[power_v1[k]])].eptr = newplist;

                            s->edgestatus[k] = VISITED;
                        }
                    }
                }

                tetorthocenter(ta[0], ta[1], ta[2], ta[3], cc, &cond);

                /* cc is the displacement of orthocenter from ta[0] */
                /* cond is the denominator ( orient2d ) value        */
                if (cond != 0) /* ignore them if cond = 0 */
                {
                    s->vv = (Coord*) malloc(sizeof(Coord) * 3);

                    for (k = 0; k < 3; k++)
                    {
                        s->vv[k] = ta[0][k] + cc[k];
                    }

                    s->status = VV;

                }
                else   /* if cond=0, s is SLIVER */
                {
                    fprintf(DFILE, "sliver!\n");
                    s->vv = NULL;
                    s->status = SLV;
                }
            }
            else   /* if on conv hull, ignore */
            {
                s->vv = NULL;
                s->status = CNV;
            }

            return NULL;
        }

        void* PowerCrustHull::compute_3d_power_edges(simplex* s, void* p)
        {

            static out_func* out_func_here;
            point v[MAXDIM];
            int j, k, inf = 0, numedges, ns, l, nedge0, nedge1, nremv, nnextv, l1, l2, nk;
            site edge0, edge1, nextv, remv, prevv;
            double ta[4][4], r1, r2, d, e;
            simplex* prevs, *nexts;

            if (p)
            {
                out_func_here = (out_func*)p;

                if (!s)
                {
                    return NULL;
                }
            }


            if ((s->status == CNV) || (s->status == SLV))
            {
                return NULL;    /* skip inf faces */
            }

            for (j = 0; j < cdim; j++)
            {
                v[j] = s->neigh[j].vert;

                for (k = 0; k < 4; k++)
                {
                    ta[j][k] = v[j][k] / pc->mult_up; /* restore original coords   */
                }
            }

            if (!inf)
            {
                for (k = 0; k < 6; k++) /* for each edge */
                {
                    if (s->edgestatus[k] == FIRST_EDGE) /* not visited edge */
                    {

                        /* check the dihedral angle */
                        d = sqdist(ta[power_v1[k]], ta[power_v2[k]]);
                        r1 = SQ(ta[power_v1[k]][0]) + SQ(ta[power_v1[k]][1]) +
                             SQ(ta[power_v1[k]][2]) - ta[power_v1[k]][3];
                        r2 = SQ(ta[power_v2[k]][0]) + SQ(ta[power_v2[k]][1]) +
                             SQ(ta[power_v2[k]][2]) - ta[power_v2[k]][3];
                        e = 2 * sqrt(r1) * sqrt(r2);

                        if ((d >= (r1 + r2 + e)) || ((d - r1 - r2) / e > pc->theta))
                        {
                            /* fprintf(DFILE,"%f\n",(d-r1-r2)/e);*/
                            /* edge0, edge1 are the vertices of the edge */
                            edge0 = s->neigh[power_v1[k]].vert;
                            edge1 = s->neigh[power_v2[k]].vert;
                            nextv = s->neigh[power_v3[k]].vert;
                            /* nextv is the opposite vtx of the next simplex */
                            remv = s->neigh[power_v4[k]].vert;
                            /* remv is a vtx of the next simplex with edge0, edge1 */
                            prevv = remv;
                            /* prevv is the vtx shared by prevs and nexts besides edge0, edge1 */

                            /* construct its dual power face */
                            s->edgestatus[k] = POW;

                            /* visit the next simplex */
                            /* print orthocenter of s->neigh[v3[k]].simp ...*/
                            prevs = s;
                            nexts = s->neigh[power_v3[k]].simp;

                            ns = power_v3[k];
                            numedges = 0;

                            while (nexts != s)
                            {
                                if (nexts->status == CNV)
                                {
                                    fprintf(DFILE, "inf reg face\n");
                                    break;
                                }
                                else
                                {
                                    fprintf(pc->PC, "%f %f %f\n", prevs->vv[0], prevs->vv[1], prevs->vv[2]);
                                    numedges++;
                                    power_numvtxs++;

                                    /* find edgenumber k of nexts for this edge */
                                    for (l = 0; l < 4; l++)
                                    {
                                        if (nexts->neigh[l].vert == edge0)
                                        {
                                            /* l == v1[k] */
                                            nedge0 = l;
                                            continue;
                                        }
                                        else if (nexts->neigh[l].vert == edge1)
                                        {
                                            /* l == v2[k] */
                                            nedge1 = l;
                                            continue;
                                        }
                                        else if (nexts->neigh[l].vert == prevv)
                                        {
                                            nremv = l;
                                            continue;
                                        }
                                        else if (nexts->neigh[l].vert == nextv)
                                        {
                                            nnextv = l;
                                            continue;

                                        }
                                        else
                                        {
                                            nnextv = l;
                                        }
                                    }

                                    if (nedge0 > nedge1)
                                    {
                                        l1 = nedge1;
                                        l2 = nedge0;
                                    }
                                    else
                                    {
                                        l2 = nedge1;
                                        l1 = nedge0;
                                    }

                                    if (l1 == 0)
                                    {
                                        if (l2 == 1)
                                        {
                                            nk = 0;
                                        }
                                        else if (l2 == 2)
                                        {
                                            nk = 1;
                                        }
                                        else
                                        {
                                            nk = 2;
                                        }
                                    }
                                    else if (l1 == 1)
                                    {
                                        if (l2 == 2)
                                        {
                                            nk = 3;
                                        }
                                        else
                                        {
                                            nk = 4;
                                        }
                                    }
                                    else
                                    {
                                        nk = 5;
                                    }

                                    /* found nk for the edge */
                                    nexts->edgestatus[nk] = POW; /* record that it's visited */
                                    /* visit next simplex (opposite vertex ns )*/
                                    prevs = nexts;
                                    prevv = nexts->neigh[nnextv].vert;
                                    nexts = nexts->neigh[nremv].simp;
                                }
                            }

                            fprintf(pc->PC, "%f %f %f\n", prevs->vv[0], prevs->vv[1], prevs->vv[2]);
                            numedges++;
                            power_numvtxs++;
                            fprintf(pc->PNF, "%d ", numedges);

                            for (l = numedges; l > 0; l--)
                            {
                                fprintf(pc->PNF, "%d ", power_numvtxs - l);
                            }

                            fprintf(pc->PNF, "\n");
                            power_numfaces++;
                        }
                        else
                        {
                            s->edgestatus[k] = NOT_POW;
                        }
                    }         /* skip if the edge is visited before */
                }
            }

            /* ignore inf faces */

            return NULL;

        }

        /* the function for computing the medial axis */

        void* PowerCrustHull::compute_axis(simplex* s, void* p)
        {

            static out_func* out_func_here;
            point v[MAXDIM];
            point  point1, point2;
            int pindex, qindex;
            int edgedata[6];
            int indices[6]; /* store the indices */
            int j, k, inf = 0;

            double ta[4][4];

            if (p)
            {
                out_func_here = (out_func*)p;

                if (!s)
                {
                    return NULL;
                }
            }


            if ((s->status == CNV) || (s->status == SLV))
            {
                return NULL;    /* skip inf faces */
            }

            for (j = 0; j < cdim; j++)
            {
                v[j] = s->neigh[j].vert;

                for (k = 0; k < 4; k++)
                {
                    ta[j][k] = v[j][k] / pc->mult_up; /* restore original coords   */
                }
            }

            if (!inf)
            {
                for (k = 0; k < 6; k++) /* for each edge */
                {
                    edgedata[k] = 0;

                    if ((s->edgestatus[k] != POW))  /* not dual to a power  face  */
                    {



                        point1 = v[power_v1[k]];
                        point2 = v[power_v2[k]];
                        pindex = pc->site_numm(point1);
                        qindex = pc->site_numm(point2);

                        if (pc->adjlist[pindex].label == HULL_IN && pc->adjlist[qindex].label == HULL_IN)
                        {
                            if (s->edgestatus[k] != ADDAXIS)
                            {
                                pc->num_axedgs++;
                                fprintf(pc->AXIS, "2 %d %d \n ", pindex, qindex);
                            }

                            edgedata[k] = VALIDEDGE;
                            indices[power_v1[k]] = pindex ;
                            indices[power_v2[k]] = qindex ;
                            s->edgestatus[k] = ADDAXIS;


                        }

                        /* now start adding triangles if present */
                    }
                }

                if ((edgedata[0] == VALIDEDGE) && (edgedata[1] == VALIDEDGE)
                    && (edgedata[3] == VALIDEDGE))
                {

                    fprintf(pc->AXIS, "3 %d %d %d \n", indices[power_v1[0]],
                            indices[power_v2[1]], indices[power_v1[3]]);
                    fprintf(pc->AXISFACE, "3 %d %d %d \n", indices[power_v1[0]],
                            indices[power_v2[1]], indices[power_v1[3]]);
                    pc->num_axedgs++;
                    pc->num_axfaces++;
                }

                if ((edgedata[1] == VALIDEDGE) && (edgedata[2] == VALIDEDGE)
                    && (edgedata[5] == VALIDEDGE))
                {
                    fprintf(pc->AXIS, "3 %d %d %d \n", indices[power_v1[1]],
                            indices[power_v2[2]], indices[power_v1[5]]);
                    fprintf(pc->AXISFACE, "3 %d %d %d \n", indices[power_v1[1]],
                            indices[power_v2[2]], indices[power_v1[5]]);
                    pc->num_axedgs++;
                    pc->num_axfaces++;

                }

                if ((edgedata[0] == VALIDEDGE) && (edgedata[2] == VALIDEDGE)
                    && (edgedata[4] == VALIDEDGE))
                {
                    fprintf(pc->AXIS, "3 %d %d %d \n", indices[power_v1[0]],
                            indices[power_v2[2]], indices[power_v1[4]]);
                    fprintf(pc->AXISFACE, "3 %d %d %d \n", indices[power_v1[0]],
                            indices[power_v2[2]], indices[power_v1[4]]);
                    pc->num_axedgs++;
                    pc->num_axfaces++;
                }

                if ((edgedata[3] == VALIDEDGE) && (edgedata[4] == VALIDEDGE)
                    && (edgedata[5] == VALIDEDGE))
                {
                    fprintf(pc->AXIS, "3 %d %d %d \n", indices[power_v1[3]],
                            indices[power_v2[4]], indices[power_v1[5]]);
                    fprintf(pc->AXISFACE, "3 %d %d %d \n", indices[power_v1[3]],
                            indices[power_v2[4]], indices[power_v1[5]]);
                    pc->num_axedgs++;
                    pc->num_axfaces++;
                }





            }

            return NULL;
        }







        /* To print out powercrust faces */
        void PowerCrustHull::construct_face(simplex* s, short k)
        {
            site edge0, edge1, nextv, remv, prevv, outsite, insite;
            simplex* prevs, *nexts;
            int j, numedges, l1, l2, nk, l, ns, nedge0, nedge1, nremv, nnextv, i;
            char cface[200];
            char indface[1024][32];  /* the indices of the face */

            double plane[3][3];
            double outpole[3], inpole[3];

            cface[0] = '\0';
            edge0 = s->neigh[power_v1[k]].vert;
            edge1 = s->neigh[power_v2[k]].vert;

            if (pc->adjlist[pc->site_numm(edge0)].label == HULL_OUT)
            {
                outsite = edge0;
                insite = edge1;
            }
            else
            {
                outsite = edge1;
                insite = edge0;
            }

            for (j = 0; j < 3; j++)
            {
                outpole[j] = outsite[j] / pc->mult_up;
                inpole[j] = insite[j] / pc->mult_up;
            }


            nextv = s->neigh[power_v3[k]].vert;
            /* nextv is the opposite vtx of the next simplex */
            remv = s->neigh[power_v4[k]].vert;
            /* remv is a vtx of the next simplex with edge0, edge1 */
            prevv = remv;
            /* prevv is the vtx shared by prevs and nexts besides edge0, edge1 */

            /* construct its dual power face */
            s->edgestatus[k] = POW;

            /* visit the next simplex */
            /* print orthocenter of s->neigh[v3[k]].simp ...*/
            prevs = s;
            nexts = s->neigh[power_v3[k]].simp;

            ns = power_v3[k];
            numedges = 0;

            while (nexts != s)
            {
                if (nexts->status == CNV)
                {
                    fprintf(DFILE, "inf reg face\n");
                    break;
                }
                else
                {
                    if (prevs->status != POLE_OUTPUT)
                    {
                        /* this vertex is not yet output */
                        prevs->status = POLE_OUTPUT;
                        prevs->poleindex = pc->num_vtxs++;
                        fprintf(pc->PC, "%f %f %f\n", prevs->vv[0], prevs->vv[1], prevs->vv[2]);
                    }

                    if (numedges < 3)
                    {
                        plane[numedges][0] = prevs->vv[0];
                        plane[numedges][1] = prevs->vv[1];
                        plane[numedges][2] = prevs->vv[2];
                    }

                    sprintf(indface[numedges], "%ld ", prevs->poleindex);
                    /*   strcat(cface,tempface);*/
                    numedges++;

                    /* find edgenumber k of nexts for this edge */
                    for (l = 0; l < 4; l++)
                    {
                        if (nexts->neigh[l].vert == edge0)
                        {
                            /* l == v1[k] */
                            nedge0 = l;
                            continue;
                        }
                        else if (nexts->neigh[l].vert == edge1)
                        {
                            /* l == v2[k] */
                            nedge1 = l;
                            continue;
                        }
                        else if (nexts->neigh[l].vert == prevv)
                        {
                            nremv = l;
                            continue;
                        }
                        else if (nexts->neigh[l].vert == nextv)
                        {
                            /*  if (nexts->neigh[nremv].simp == s) { */
                            nnextv = l;
                            continue;
                            /*}
                              else fprintf(DFILE,"cannot happen l=%d!!\n",l); */
                        }
                        else
                        {
                            nnextv = l;
                        }
                    }

                    if (nedge0 > nedge1)
                    {
                        l1 = nedge1;
                        l2 = nedge0;
                    }
                    else
                    {
                        l2 = nedge1;
                        l1 = nedge0;
                    }

                    if (l1 == 0)
                    {
                        if (l2 == 1)
                        {
                            nk = 0;
                        }
                        else if (l2 == 2)
                        {
                            nk = 1;
                        }
                        else
                        {
                            nk = 2;
                        }
                    }
                    else if (l1 == 1)
                    {
                        if (l2 == 2)
                        {
                            nk = 3;
                        }
                        else
                        {
                            nk = 4;
                        }
                    }
                    else
                    {
                        nk = 5;
                    }

                    /* found nk for the edge */
                    nexts->edgestatus[nk] = POW; /* record that it's visited */
                    /* visit next simplex (opposite vertex ns )*/
                    prevs = nexts;
                    prevv = nexts->neigh[nnextv].vert;
                    nexts = nexts->neigh[nremv].simp;
                }
            }

            if (prevs->status != POLE_OUTPUT)
            {
                prevs->status = POLE_OUTPUT;
                prevs->poleindex = pc->num_vtxs++;
                fprintf(pc->PC, "%f %f %f\n", prevs->vv[0], prevs->vv[1], prevs->vv[2]);
            }

            if (numedges < 3)
            {
                plane[numedges][0] = prevs->vv[0];
                plane[numedges][1] = prevs->vv[1];
                plane[numedges][2] = prevs->vv[2];

            }

            sprintf(indface[numedges], "%ld ", prevs->poleindex);

            numedges++;
            fprintf(pc->PNF, "%d ", numedges);

            if (!correct_orientation(plane[0], plane[1], plane[2], inpole, outpole))
                for (i = numedges - 1; i >= 0; i--)
                {
                    fprintf(pc->PNF, "%s ", indface[i]);
                }
            else
                for (i = 0; i < numedges; i++)
                {
                    fprintf(pc->PNF, "%s ", indface[i]);
                }

            fprintf(pc->PNF, "\n");

            pc->num_faces++;
        }



        int PowerCrustHull::correct_orientation(double* p1, double* p2, double* p3, double* inp, double* outp)
        {

            double normal[3];
            double v1[3], v2[3];
            double xcross, ycross, zcross;
            int numplus = 0, numminus = 0;

            normal[0] = outp[0] - inp[0];
            normal[1] = outp[1] - inp[1];
            normal[2] = outp[2] - inp[2];

            v1[0] = p2[0] - p1[0];
            v1[1] = p2[1] - p1[1];
            v1[2] = p2[2] - p1[2];

            v2[0] = p3[0] - p2[0];
            v2[1] = p3[1] - p2[1];
            v2[2] = p3[2] - p2[2];

            xcross = v1[1] * v2[2] - v1[2] * v2[1];
            ycross = v1[2] * v2[0] - v1[0] * v2[2];
            zcross = v1[0] * v2[1] - v1[1] * v2[0];

            if ((xcross * normal[0]) > 0)
            {
                numplus++;
            }
            else
            {
                numminus++;
            }


            if ((ycross * normal[1]) > 0)
            {
                numplus++;
            }
            else
            {
                numminus++;
            }


            if ((zcross * normal[2]) > 0)
            {
                numplus++;
            }
            else
            {
                numminus++;
            }

            if (numplus > numminus)
            {
                return 1;
            }
            else
            {
                return 0;
            }

        }

    }
}

#ifdef WIN32
#pragma warning(pop)
#endif

