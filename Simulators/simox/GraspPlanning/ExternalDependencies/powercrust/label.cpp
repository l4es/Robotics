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


namespace GraspStudio
{

    namespace PowerCrust
    {
        int PowerCrustHull::propagate()
        {
            int pid;

            pid = extract_max();

            if (pc->adjlist[pid].in > pc->adjlist[pid].out)
            {
                pc->adjlist[pid].label = HULL_IN;
            }
            else
            {
                pc->adjlist[pid].label = HULL_OUT;
            }

            /*  fprintf(DFILE,"pole %d in %f out %f label %d\n",pid, pc->adjlist[pid].in, pc->adjlist[pid].out, pc->adjlist[pid].label);  */

            if (pid != -1)
            {
                /*    fprintf(DFILE,"propagating pole %d..\n",pid);  */
                opp_update(pid);
                sym_update(pid);
            }

            return pid;
        }

        void PowerCrustHull::opp_update(int pi)
        {
            struct plist* pindex;
            int npi, nhi;
            double temp;

            pindex = pc->opplist[pi];

            while (pindex != NULL)
            {
                npi = pindex->pid;

                if (pc->defer)
                {
                    if (pc->adjlist[npi].bad == BAD_POLE)
                    {
                        /*  fprintf(DFILE,"found bad pole.. defer its labeling\n"); */
                        pindex = pindex->next;
                        continue;
                    }
                }

                if (pc->adjlist[npi].label == HULL_INIT)   /* not yet labeled */
                {
                    if (pc->adjlist[npi].hid == 0)   /* not in the heap */
                    {
                        if (pc->adjlist[pi].in > pc->adjlist[pi].out)
                        {
                            /* propagate in*cos to out */
                            pc->adjlist[npi].out = (-1.0) * pc->adjlist[pi].in * pindex->angle;
                            /*  fprintf(DFILE,"pole %d.out = %f\n",npi,pc->adjlist[npi].out); */
                            insert_heap(npi, pc->adjlist[npi].out);
                        }
                        else if (pc->adjlist[pi].in < pc->adjlist[pi].out)
                        {
                            /* propagate out*cos to in */
                            pc->adjlist[npi].in = (-1.0) * pc->adjlist[pi].out * pindex->angle;
                            /* fprintf(DFILE,"pole %d.in = %f\n",npi,pc->adjlist[npi].in); */
                            insert_heap(npi, pc->adjlist[npi].in);
                        }
                    }
                    else   /* in the heap */
                    {
                        nhi = pc->adjlist[npi].hid;

                        if (pc->adjlist[pi].in > pc->adjlist[pi].out)
                        {
                            /* propagate in*cos to out */
                            temp = (-1.0) * pc->adjlist[pi].in * pindex->angle;

                            if (temp > pc->adjlist[npi].out)
                            {
                                pc->adjlist[npi].out = temp;
                                update_pri(nhi, npi);
                            }
                        }
                        else if (pc->adjlist[pi].in < pc->adjlist[pi].out)
                        {
                            /* propagate out*cos to in */
                            temp = (-1.0) * pc->adjlist[pi].out * pindex->angle;

                            if (temp > pc->adjlist[npi].in)
                            {
                                pc->adjlist[npi].in = temp;
                                update_pri(nhi, npi);
                            }
                        }
                    }
                }

                pindex = pindex->next;
            }
        }

        void PowerCrustHull::sym_update(int pi)
        {
            struct edgesimp* eindex;
            int npi, nhi;
            double temp;

            eindex = pc->adjlist[pi].eptr;

            while (eindex != NULL)
            {
                npi = eindex->pid;

                if (pc->defer)
                {
                    if (pc->adjlist[npi].bad == BAD_POLE)
                    {
                        eindex = eindex->next;
                        /*fprintf(DFILE,"found bad pole.. defer its labeling\n");*/
                        continue;
                    }
                }

                /* try to label deeply intersecting unlabeled neighbors */
                if ((pc->adjlist[npi].label == HULL_INIT) && (eindex->angle > pc->theta))
                {
                    /* not yet labeled */
                    if (pc->adjlist[npi].hid == 0)   /* not in the heap */
                    {
                        if (pc->adjlist[pi].in > pc->adjlist[pi].out)
                        {
                            /* propagate in*cos to in */
                            pc->adjlist[npi].in = pc->adjlist[pi].in * eindex->angle;
                            insert_heap(npi, pc->adjlist[npi].in);
                        }
                        else if (pc->adjlist[pi].in < pc->adjlist[pi].out)
                        {
                            /* propagate out*cos to out */
                            pc->adjlist[npi].out = pc->adjlist[pi].out * eindex->angle;
                            insert_heap(npi, pc->adjlist[npi].out);
                        }
                    }
                    else   /* in the heap */
                    {
                        if (heap_A[pc->adjlist[npi].hid].pid != npi)
                        {
                            fprintf(DFILE, "ERROR\n");
                        }

                        nhi = pc->adjlist[npi].hid;

                        if (pc->adjlist[pi].in > pc->adjlist[pi].out)
                        {
                            /* propagate in*cos to in */
                            temp = pc->adjlist[pi].in * eindex->angle;

                            if (temp > pc->adjlist[npi].in)
                            {
                                pc->adjlist[npi].in = temp;
                                update_pri(nhi, npi);
                            }
                        }
                        else if (pc->adjlist[pi].in < pc->adjlist[pi].out)
                        {
                            /* propagate out*cos to out */
                            temp = pc->adjlist[pi].out * eindex->angle;

                            if (temp > pc->adjlist[npi].out)
                            {
                                pc->adjlist[npi].out = temp;
                                update_pri(nhi, npi);
                            }
                        }
                    }
                }

                eindex = eindex->next;
            }
        }

        void PowerCrustHull::update_pri(int hi, int pi)
        /* update heap_A[hi].pri using pc->adjlist[pi].in/out */
        {
            double pr;

            if ((heap_A[hi].pid != pi) || (pc->adjlist[pi].hid != hi))
            {
                fprintf(DFILE, "Error update_pri!\n");
                return;
            }

            if (pc->adjlist[pi].in == 0.0)
            {
                pr = pc->adjlist[pi].out;
            }
            else if (pc->adjlist[pi].out == 0.0)
            {
                pr = pc->adjlist[pi].in;
            }
            else   /* both in/out nonzero */
            {
                if (pc->adjlist[pi].in > pc->adjlist[pi].out)
                {
                    pr =  pc->adjlist[pi].in - pc->adjlist[pi].out - 1;
                }
                else
                {
                    pr = pc->adjlist[pi].out - pc->adjlist[pi].in - 1;
                }
            }

            update(hi, pr);
        }

        void PowerCrustHull::label_unlabeled(int num)
        {
            struct plist* pindex;
            struct edgesimp* eindex;
            int npi, i, opplabel;
            int tlabel;
            double tangle, tangle1;


            for (i = 0; i < num; i++)
            {
                if (pc->adjlist[i].label == HULL_INIT)   /* pole i is unlabeled.. try to label now */
                {
                    tlabel = HULL_INIT;
                    opplabel = HULL_INIT;
                    pindex = pc->opplist[i];

                    if ((pindex == NULL) && (pc->adjlist[i].eptr == NULL))
                    {
                        fprintf(DFILE, "no opp pole, no adjacent pole!\n");
                        continue;
                    }

                    /* check whether there is opp pole */
                    while (pindex != NULL) /* opp pole */
                    {
                        npi = pindex->pid;

                        if (pc->adjlist[npi].label != HULL_INIT)
                        {
                            fprintf(DFILE, "opp is labeled\n");

                            if (opplabel == HULL_INIT)
                            {
                                opplabel = pc->adjlist[npi].label;
                            }
                            else if (opplabel != pc->adjlist[npi].label)
                            {
                                /* opp poles have different labels ... inconsistency! */
                                fprintf(DFILE, "opp poles have inconsistent labels\n");
                                opplabel = HULL_INIT; /* ignore the label of opposite poles */
                            }
                        }

                        pindex = pindex->next;
                    }

                    tangle = -3.0;
                    tangle1 = -3.0;
                    eindex = pc->adjlist[i].eptr;

                    while (eindex != NULL)
                    {
                        npi = eindex->pid;

                        if (pc->adjlist[npi].label == HULL_IN)
                        {
                            if (tangle < eindex->angle)
                            {
                                tangle = eindex->angle;
                            }
                        }
                        else if (pc->adjlist[npi].label == HULL_OUT)
                        {
                            if (tangle1 < eindex->angle)
                            {
                                tangle1 = eindex->angle;
                            }
                        }

                        eindex = eindex->next;
                    }

                    /* now tangle, tangle 1 are angles of most deeply interesecting in, out poles */
                    if (tangle == -3.0)   /* there was no in poles */
                    {
                        if (tangle1 == -3.0)   /* there was no out poles */
                        {
                            if (opplabel == HULL_INIT)   /* cannot trust opp pole or no labeled opp pole */
                            {
                                fprintf(DFILE, "1: cannot label pole %d\n", i);
                            }
                            else if (opplabel == HULL_IN)
                            {
                                pc->adjlist[i].label = HULL_OUT;
                            }
                            else
                            {
                                pc->adjlist[i].label = HULL_IN;
                            }
                        }
                        else if (tangle1 > pc->deep)   /* interesecting deeply only out poles */
                        {
                            pc->adjlist[i].label = HULL_OUT;
                        }
                        else   /* no deeply intersecting poles . use opp pole */
                        {
                            if (opplabel == HULL_INIT)   /* cannot trust opp pole or no labeled opp pole */
                            {
                                fprintf(DFILE, "2: cannot label pole %d\n", i);
                            }
                            else if (opplabel == HULL_IN)
                            {
                                pc->adjlist[i].label = HULL_OUT;
                            }
                            else
                            {
                                pc->adjlist[i].label = HULL_IN;
                            }
                        }
                    }
                    else if (tangle1 == -3.0)   /* there are in pole but no out pole */
                    {
                        if (tangle > pc->deep)   /* interesecting deeply only in poles */
                        {
                            pc->adjlist[i].label = HULL_IN;
                        }
                        else   /* no deeply intersecting poles . use opp pole */
                        {
                            if (opplabel == HULL_INIT)   /* cannot trust opp pole or no labeled opp pole */
                            {
                                fprintf(DFILE, "2: cannot label pole %d\n", i);
                            }
                            else if (opplabel == HULL_IN)
                            {
                                pc->adjlist[i].label = HULL_OUT;
                            }
                            else
                            {
                                pc->adjlist[i].label = HULL_IN;
                            }
                        }
                    }
                    else   /* there are both in/out poles */
                    {
                        if (tangle > pc->deep)
                        {
                            if (tangle1 > pc->deep)   /* intersecting both deeply */
                            {
                                /* use opp */
                                if (opplabel == HULL_INIT)   /* cannot trust opp pole or no labeled opp pole */
                                {
                                    /* then give label with bigger angle */
                                    fprintf(DFILE, "intersect both deeply but no opp,in %f out %f.try to label more deeply intersected label.\n", tangle, tangle1);

                                    if (tangle > tangle1)
                                    {
                                        pc->adjlist[i].label = HULL_IN;
                                    }
                                    else
                                    {
                                        pc->adjlist[i].label = HULL_OUT;
                                    }
                                }
                                else if (opplabel == HULL_IN)
                                {
                                    pc->adjlist[i].label = HULL_OUT;
                                }
                                else
                                {
                                    pc->adjlist[i].label = HULL_IN;
                                }
                            }
                            else   /* intersecting only in deeply */
                            {
                                pc->adjlist[i].label = HULL_IN;
                            }
                        }
                        else if (tangle1 > pc->deep)   /* intersecting only out deeply */
                        {
                            pc->adjlist[i].label = HULL_OUT;
                        }
                        else   /* no deeply intersecting poles . use opp pole */
                        {
                            if (opplabel == HULL_INIT)   /* cannot trust opp pole or no labeled opp pole */
                            {
                                fprintf(DFILE, "3: cannot label pole %d\n", i);
                            }
                            else if (opplabel == HULL_IN)
                            {
                                pc->adjlist[i].label = HULL_OUT;
                            }
                            else
                            {
                                pc->adjlist[i].label = HULL_IN;
                            }
                        }
                    }




                    /* no labeled opp pole - label pole same as the most deeply intersecting labeled pole ... no longer needed because opp values are already propagated..
                       tangle = -3.0;
                       eindex = pc->adjlist[i].eptr;
                       while (eindex != NULL) {
                       npi = eindex->pid;
                       if (pc->adjlist[npi].label == IN) {
                       if (tangle < eindex->angle) {
                       tangle = eindex->angle;
                       tlabel = IN;
                       }
                       }
                       else if (pc->adjlist[npi].label == OUT) {
                       if (tangle < eindex->angle) {
                       tangle = eindex->angle;
                       tlabel = OUT;
                       }
                       }
                       eindex = eindex->next;
                       }
                       fprintf(DFILE,"pole %d  max angle %f label %d\n", i,tangle,tlabel);
                       pc->adjlist[i].label = tlabel;
                    */


                }
            }
        }

    }
}
