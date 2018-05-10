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


        double PowerCrustHull::SQ(double a)
        {
            return a * a;
        }

        void PowerCrustHull::init_heap(int num, FILE* DFILE)
        {
            heap_A = (struct heap_array*)calloc(num, sizeof(struct heap_array));
            heap_size = 0;
            heap_length = num;

            if (!DFILE)
            {
                DFILE = stdout;
            }

            fprintf(DFILE, "heap %d initialized\n", num);
        }
        /* for priority queue */
        int PowerCrustHull::LEFT(int i)
        {
            return (i) * 2;
        }
        int PowerCrustHull::RIGHT(int i)
        {
            return (i) * 2 + 1;
        }
        int PowerCrustHull::PARENT(int i)
        {
            return (i) / 2;
        }


        void PowerCrustHull::heapify(int hi)
        {
            int largest;
            int temp;
            double td;

            if ((LEFT(hi) <= heap_size) && (heap_A[LEFT(hi)].pri > heap_A[hi].pri))
            {
                largest = LEFT(hi);
            }
            else
            {
                largest = hi;
            }

            if ((RIGHT(hi) <= heap_size) && (heap_A[RIGHT(hi)].pri > heap_A[largest].pri))
            {
                largest = RIGHT(hi);
            }

            if (largest != hi)
            {
                temp = heap_A[hi].pid;
                heap_A[hi].pid = heap_A[largest].pid;
                pc->adjlist[heap_A[hi].pid].hid = hi;
                heap_A[largest].pid = temp;
                pc->adjlist[heap_A[largest].pid].hid = largest;
                td =  heap_A[hi].pri;
                heap_A[hi].pri = heap_A[largest].pri;
                heap_A[largest].pri = td;
                heapify(largest);
            }

        }

        int PowerCrustHull::extract_max()
        {
            int max;

            if (heap_size < 1)
            {
                return -1;
            }

            max = heap_A[1].pid;
            heap_A[1].pid = heap_A[heap_size].pid;
            heap_A[1].pri = heap_A[heap_size].pri;
            pc->adjlist[heap_A[1].pid].hid = 1;
            heap_size--;
            heapify(1);
            return max;
        }

        int PowerCrustHull::insert_heap(int pi, double pr)
        {
            int i;

            heap_size++;
            /*printf("heap_size %d\n",heap_size); */
            i = heap_size;

            while ((i > 1) && (heap_A[PARENT(i)].pri < pr))
            {
                heap_A[i].pid = heap_A[PARENT(i)].pid;
                heap_A[i].pri = heap_A[PARENT(i)].pri;
                pc->adjlist[heap_A[i].pid].hid = i;
                i = PARENT(i);
            };

            heap_A[i].pri = pr;

            heap_A[i].pid = pi;

            pc->adjlist[pi].hid = i;

            return i;
        }

        void PowerCrustHull::update(int hi, double pr)
        /* make the element heap_A[hi].pr = pr ... */
        {
            int pi, i;

            heap_A[hi].pri = pr;
            pi = heap_A[hi].pid;

            if (pr > heap_A[PARENT(hi)].pri)
            {
                i = hi;

                while ((i > 1) && (heap_A[PARENT(i)].pri < pr))
                {
                    heap_A[i].pid = heap_A[PARENT(i)].pid;
                    heap_A[i].pri = heap_A[PARENT(i)].pri;
                    pc->adjlist[heap_A[i].pid].hid = i;
                    i = PARENT(i);
                };

                heap_A[i].pri = pr;

                heap_A[i].pid = pi;

                pc->adjlist[pi].hid = i;
            }
            else
            {
                heapify(hi);
            }
        }

    }

}


