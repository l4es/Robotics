/* hull.c : "combinatorial" functions for hull computation */

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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <assert.h>
#include <string.h>

#include "hull.h"
#include "powercrust.h"


//#define HULL_DEBUG_TO_FILE
//#define HULL_DEBUG_TO_FILE_FILENAME "c:\\tmp\\powercrust\\1\\powercrust_hull_simox.txt"


#ifdef HULL_DEBUG_TO_FILE
#include "CustomLogger.h"
CustomLogger logHull(CustomLogger::eFile, HULL_DEBUG_TO_FILE_FILENAME);
#endif


namespace GraspStudio
{

    namespace PowerCrust
    {




        const int max_blocks = 10000;
        const int Nobj = 10000;

#define POWERCRUST_STORAGE_GLOBALS(X)       \
    \
    extern size_t X##_size;         \
    extern X *X##_list;         \
    extern X *new_block_##X(int);       \
    extern void flush_##X##_blocks(void);   \
    void free_##X##_storage(void);      \
     

#define POWERCRUST_INCP(X,p,k) ((X*) ( (char*)p + (k) * X##_size)) /* portability? */


#define POWERCRUST_STORAGE(X)                       \
    \
    size_t  X##_size;                       \
    X   *X##_list = 0;                      \
    \
    X *new_block_##X(int make_blocks)               \
    {   int i;                          \
        static  X *X##_block_table[max_blocks];         \
        X *xlm, *xbt;                   \
        static int num_##X##_blocks;                \
        if (make_blocks) {                  \
            assert(num_##X##_blocks<max_blocks);        \
            xbt = X##_block_table[num_##X##_blocks++] = (X*)malloc(Nobj * X##_size); \
            memset(xbt,0,Nobj * X##_size);  \
            assert(xbt);                    \
            \
            xlm = POWERCRUST_INCP(X,xbt,Nobj);              \
            for (i=0;i<Nobj; i++) {             \
                xlm = POWERCRUST_INCP(X,xlm,(-1));          \
                xlm->next = X##_list;           \
                X##_list = xlm;             \
            }                       \
            \
            return X##_list;                \
        }                           \
        \
        for (i=0; i<num_##X##_blocks; i++)          \
            free(X##_block_table[i]);           \
        num_##X##_blocks = 0;                   \
        X##_list = 0;                       \
        return 0;                       \
    }                               \
    \
    void free_##X##_storage(void) {new_block_##X(0);}       \
    /*end of STORAGE*/

#define POWERCRUST_NEWL(X,p)                        \
    {                               \
        p = X##_list ? X##_list : new_block_##X(1);     \
        assert(p);                      \
        X##_list = p->next;                 \
    }                               \
     


#define POWERCRUST_NEWLRC(X,p)                      \
    {                               \
        p = X##_list ? X##_list : new_block_##X(1);     \
        assert(p);                      \
        X##_list = p->next;                 \
        p->ref_count = 1;                   \
    }                               \
     

#define POWERCRUST_FREEL(X,p)                       \
    {                               \
        memset((p),0,X##_size);                 \
        (p)->next = X##_list;                   \
        X##_list = p;                       \
    }                               \
     

#define POWERCRUST_dec_ref(X,v) {if ((v) && --(v)->ref_count == 0) POWERCRUST_FREEL(X,(v));}
#define POWERCRUST_inc_ref(X,v) {if (v) v->ref_count++;}
#define POWERCRUST_NULLIFY(X,v) {POWERCRUST_dec_ref(X,v); v = NULL;}



#define POWERCRUST_mod_refs(op,s)                   \
    {                           \
        int i;                      \
        neighbor *mrsn;                 \
        \
        for (i=-1,mrsn=s->neigh-1;i<cdim;i++,mrsn++)    \
            POWERCRUST_##op##_ref(basis_s, mrsn->basis);        \
    }

#define POWERCRUST_free_simp(s)             \
    {   POWERCRUST_mod_refs(dec,s);         \
        POWERCRUST_FREEL(basis_s,s->normal);        \
        POWERCRUST_FREEL(simplex, s);           \
    }                       \
     

#define POWERCRUST_copy_simp(n,s)           \
    {   POWERCRUST_NEWL(simplex,n);         \
        memcpy(n,s,simplex_size);       \
        POWERCRUST_mod_refs(inc,s);         \
    }                       \
     

        //////////////////////////////////////////////////////////////////////////
        // from ndefs.h
        //////////////////////////////////////////////////////////////////////////

        POWERCRUST_STORAGE_GLOBALS(simplex)
        POWERCRUST_STORAGE_GLOBALS(basis_s)



        //////////////////////////////////////////////////////////////////////////
        // from hull.h
        //////////////////////////////////////////////////////////////////////////
        POWERCRUST_STORAGE_GLOBALS(fg)
        POWERCRUST_STORAGE_GLOBALS(Tree)


        //////////////////////////////////////////////////////////////////////////
        // from fg.c
        //////////////////////////////////////////////////////////////////////////
        /*
                   An implementation of top-down splaying with sizes
                     D. Sleator <sleator@cs.cmu.edu>, January 1994.

          This extends top-down-splay.c to maintain a size field in each node.
          This is the number of nodes in the subtree rooted there.  This makes
          it possible to efficiently compute the rank of a key.  (The rank is
          the number of nodes to the left of the given key.)  It it also
          possible to quickly find the node of a given rank.  Both of these
          operations are illustrated in the code below.  The remainder of this
          introduction is taken from top-down-splay.c.

          "Splay trees", or "self-adjusting search trees" are a simple and
          efficient data structure for storing an ordered set.  The data
          structure consists of a binary tree, with no additional fields.  It
          allows searching, insertion, deletion, deletemin, deletemax,
          splitting, joining, and many other operations, all with amortized
          logarithmic performance.  Since the trees adapt to the sequence of
          requests, their performance on real access patterns is typically even
          better.  Splay trees are described in a number of texts and papers
          [1,2,3,4].

          The code here is adapted from simple top-down splay, at the bottom of
          page 669 of [2].  It can be obtained via anonymous ftp from
          spade.pc.cs.cmu.edu in directory /usr/sleator/public.

          The chief modification here is that the splay operation works even if the
          item being splayed is not in the tree, and even if the tree root of the
          tree is NULL.  So the line:

                                      t = splay(i, t);

          causes it to search for item with key i in the tree rooted at t.  If it's
          there, it is splayed to the root.  If it isn't there, then the node put
          at the root is the last one before NULL that would have been reached in a
          normal binary search for i.  (It's a neighbor of i in the tree.)  This
          allows many other operations to be easily implemented, as shown below.

          [1] "Data Structures and Their Algorithms", Lewis and Denenberg,
               Harper Collins, 1991, pp 243-251.
          [2] "Self-adjusting Binary Search Trees" Sleator and Tarjan,
               JACM Volume 32, No 3, July 1985, pp 652-686.
          [3] "Data Structure and Algorithm Analysis", Mark Weiss,
               Benjamin Cummins, 1992, pp 119-130.
          [4] "Data Structures, Algorithms, and Performance", Derick Wood,
               Addison-Wesley, 1993, pp 367-375
        */


        POWERCRUST_STORAGE(Tree)


#define compare(i,j) (pc->site_numm(i)- pc->site_numm(j))
        /* This is the comparison.                                       */
        /* Returns <0 if i<j, =0 if i=j, and >0 if i>j                   */

#define node_size(x) ((x) ? ((x)->size) : 0 )
        /* This macro returns the size of a node.  Unlike "x->size",     */
        /* it works even if x=NULL.  The test could be avoided by using  */
        /* a special version of NULL which was a real node with size 0.  */

        Tree* PowerCrustHull::splay(site i, Tree* t)
        /* Splay using the key i (which may or may not be in the tree.) */
        /* The starting root is t, and the tree used is defined by rat  */
        /* size fields are maintained */
        {
            Tree N, *l, *r, *y;
            int comp, root_size, l_size, r_size;

            if (!t)
            {
                return t;
            }

            N.left = N.right = NULL;
            l = r = &N;
            root_size = node_size(t);
            l_size = r_size = 0;

            for (;;)
            {
                comp = compare(i, t->key);

                if (comp < 0)
                {
                    if (!t->left)
                    {
                        break;
                    }

                    if (compare(i, t->left->key) < 0)
                    {
                        y = t->left;                           /* rotate right */
                        t->left = y->right;
                        y->right = t;
                        t->size = node_size(t->left) + node_size(t->right) + 1;
                        t = y;

                        if (!t->left)
                        {
                            break;
                        }
                    }

                    r->left = t;                               /* link right */
                    r = t;
                    t = t->left;
                    r_size += 1 + node_size(r->right);
                }
                else if (comp > 0)
                {
                    if (!t->right)
                    {
                        break;
                    }

                    if (compare(i, t->right->key) > 0)
                    {
                        y = t->right;                          /* rotate left */
                        t->right = y->left;
                        y->left = t;
                        t->size = node_size(t->left) + node_size(t->right) + 1;
                        t = y;

                        if (!t->right)
                        {
                            break;
                        }
                    }

                    l->right = t;                              /* link left */
                    l = t;
                    t = t->right;
                    l_size += 1 + node_size(l->left);
                }
                else
                {
                    break;
                }
            }

            l_size += node_size(t->left);  /* Now l_size and r_size are the sizes of */
            r_size += node_size(t->right); /* the left and right trees we just built.*/
            t->size = l_size + r_size + 1;

            l->right = r->left = NULL;

            /* The following two loops correct the size fields of the right path  */
            /* from the left child of the root and the right path from the left   */
            /* child of the root.                                                 */
            for (y = N.right; y != NULL; y = y->right)
            {
                y->size = l_size;
                l_size -= 1 + node_size(y->left);
            }

            for (y = N.left; y != NULL; y = y->left)
            {
                y->size = r_size;
                r_size -= 1 + node_size(y->right);
            }

            l->right = t->left;                                /* assemble */
            r->left = t->right;
            t->left = N.right;
            t->right = N.left;

            return t;
        }

        Tree* PowerCrustHull::insert(site i, Tree* t)
        {
            /* Insert key i into the tree t, if it is not already there. */
            /* Return a pointer to the resulting tree.                   */
            Tree* newT;

            if (t != NULL)
            {
                t = splay(i, t);

                if (compare(i, t->key) == 0)
                {
                    return t;  /* it's already there */
                }
            }

            POWERCRUST_NEWL(Tree, newT)

            if (!t)
            {
                newT->left = newT->right = NULL;
            }
            else if (compare(i, t->key) < 0)
            {
                newT->left = t->left;
                newT->right = t;
                t->left = NULL;
                t->size = 1 + node_size(t->right);
            }
            else
            {
                newT->right = t->right;
                newT->left = t;
                t->right = NULL;
                t->size = 1 + node_size(t->left);
            }

            newT->key = i;
            newT->size = 1 + node_size(newT->left) + node_size(newT->right);
            return newT;
        }

        Tree* PowerCrustHull::deleteT(site i, Tree* t)
        {
            /* Deletes i from the tree if it's there.               */
            /* Return a pointer to the resulting tree.              */
            Tree* x;
            int tsize;

            if (!t)
            {
                return NULL;
            }

            tsize = t->size;
            t = splay(i, t);

            if (compare(i, t->key) == 0)                 /* found it */
            {
                if (!t->left)
                {
                    x = t->right;
                }
                else
                {
                    x = splay(i, t->left);
                    x->right = t->right;
                }

                POWERCRUST_FREEL(Tree, t);

                if (x)
                {
                    x->size = tsize - 1;
                }

                return x;
            }
            else
            {
                return t;                         /* It wasn't there */
            }
        }

        Tree* PowerCrustHull::find_rank(int r, Tree* t)
        {
            /* Returns a pointer to the node in the tree with the given rank.  */
            /* Returns NULL if there is no such node.                          */
            /* Does not change the tree.  To guarantee logarithmic behavior,   */
            /* the node found here should be splayed to the root.              */
            int lsize;

            if ((r < 0) || (r >= node_size(t)))
            {
                return NULL;
            }

            for (;;)
            {
                lsize = node_size(t->left);

                if (r < lsize)
                {
                    t = t->left;
                }
                else if (r > lsize)
                {
                    r = r - lsize - 1;
                    t = t->right;
                }
                else
                {
                    return t;
                }
            }
        }

        void PowerCrustHull::printtree_flat_inner(Tree* t)
        {
            if (!t)
            {
                return;
            }

            printtree_flat_inner(t->right);
            printf("%f ", *(t->key));
            fflush(stdout);
            printtree_flat_inner(t->left);
        }

        void PowerCrustHull::printtree_flat(Tree* t)
        {
            if (!t)
            {
                printf("<empty tree>");
                return;
            }

            printtree_flat_inner(t);
        }


        void PowerCrustHull::printtree(Tree* t, int d)
        {
            int i;

            if (!t)
            {
                return;
            }

            printtree(t->right, d + 1);

            for (i = 0; i < d; i++)
            {
                printf("  ");
            }

            printf("%f(%d)\n", *(t->key), t->size);
            fflush(stdout);
            printtree(t->left, d + 1);
        }






        fg* faces_gr_t;

        POWERCRUST_STORAGE(fg)

#define snkey(x) pc->site_numm((x)->vert)

        fg* PowerCrustHull::find_fg(simplex* s, int q)
        {

            fg* f;
            neighbor* si, *sn = s->neigh;
            Tree* t;

            if (q == 0)
            {
                return faces_gr_t;
            }

            if (!faces_gr_t)
            {
                POWERCRUST_NEWLRC(fg, faces_gr_t);
            }

            f = faces_gr_t;

            for (si = sn; si < sn + cdim; si++) if (q & (1 << (si - sn)))
                {
                    t = f->facets = insert(si->vert, f->facets);

                    if (!t->fgs) POWERCRUST_NEWLRC(fg, (t->fgs))
                        f = t->fgs;
                }

            return f;
        }

        void* PowerCrustHull::add_to_fg(simplex* s, void* dum)
        {

            neighbor t, *si, *sj, *sn = s->neigh;
            fg* fq;
            int q, m, Q = 1 << cdim;

            /* sort neigh by site number */
            for (si = sn + 2; si < sn + cdim; si++)
                for (sj = si; sj > sn + 1 && snkey(sj - 1) > snkey(sj); sj--)
                {
                    t = *(sj - 1);
                    *(sj - 1) = *sj;
                    *sj = t;
                }

            POWERCRUST_NULLIFY(basis_s, s->normal);
            POWERCRUST_NULLIFY(basis_s, s->neigh[0].basis);

            /* insert subsets */
            for (q = 1; q < Q; q++)
            {
                find_fg(s, q);
            }

            /* include all superset relations */
            for (q = 1; q < Q; q++)
            {
                fq = find_fg(s, q);
                assert(fq);

                for (m = 1, si = sn; si < sn + cdim; si++, m <<= 1) if (!(q & m))
                    {
                        fq->facets = insert(si->vert, fq->facets);
                        fq->facets->fgs = find_fg(s, q | m);
                    }
            }

            return NULL;
        }

        fg* PowerCrustHull::build_fg(simplex* root)
        {
            faces_gr_t = 0;
            visit_hull(root, &GraspStudio::PowerCrust::PowerCrustHull::add_to_fg);
            return faces_gr_t;
        }

        void PowerCrustHull::visit_fg_i(void (PowerCrustHull::*v_fg)(Tree*, int, int),
                                        Tree* t, int depth, int vn, int boundary)
        {
            int boundaryc = boundary;

            if (!t)
            {
                return;
            }

            assert(t->fgs);

            if (t->fgs->mark != vn)
            {
                t->fgs->mark = vn;

                if (t->key != infinity && !mo[pc->site_numm(t->key)])
                {
                    boundaryc = 0;
                }

                (*this.*v_fg)(t, depth, boundaryc);
                visit_fg_i(v_fg, t->fgs->facets, depth + 1, vn, boundaryc);
            }

            visit_fg_i(v_fg, t->left, depth, vn, boundary);
            visit_fg_i(v_fg, t->right, depth, vn, boundary);
        }

        void PowerCrustHull::visit_fg(fg* faces_gr, void (PowerCrustHull::*v_fg)(Tree*, int, int))
        {
            static int fg_vn;
            visit_fg_i(v_fg, faces_gr->facets, 0, ++fg_vn, 1);
        }

        int PowerCrustHull::visit_fg_i_far(void (PowerCrustHull::*v_fg)(Tree*, int),
                                           Tree* t, int depth, int vn)
        {
            int nb = 0;

            if (!t)
            {
                return 0;
            }

            assert(t->fgs);

            if (t->fgs->mark != vn)
            {
                t->fgs->mark = vn;
                nb = (t->key == infinity) || mo[pc->site_numm(t->key)];

                if (!nb && !visit_fg_i_far(v_fg, t->fgs->facets, depth + 1, vn))
                {
                    (*this.*v_fg)(t, depth);
                }
            }

            nb = visit_fg_i_far(v_fg, t->left, depth, vn) || nb;
            nb = visit_fg_i_far(v_fg, t->right, depth, vn) || nb;
            return nb;
        }

        void PowerCrustHull::visit_fg_far(fg* faces_gr, void (PowerCrustHull::*v_fg)(Tree*, int))
        {
            static int fg_vn;
            visit_fg_i_far(v_fg, faces_gr->facets, 0, --fg_vn);
        }



        FILE* FG_OUT;

        void PowerCrustHull::p_fg(Tree* t, int depth, int bad)
        {
            static int fa[MAXDIM];
            int i;
            static double mults[MAXDIM];

            if (mults[0] == 0)
            {
                mults[pdim] = 1;

                for (i = pdim - 1; i >= 0; i--)
                {
                    mults[i] = pc->mult_up * mults[i + 1];
                }
            }

            fa[depth] = pc->site_numm(t->key);

            for (i = 0; i <= depth; i++)
            {
                fprintf(FG_OUT, "%d ", fa[i]);
            }

            fprintf(FG_OUT, "   %G\n", t->fgs->vol / mults[depth]);
        }

        int p_fg_x_depth;

        void PowerCrustHull::p_fg_x(Tree* t, int depth, int bad)
        {

            static int fa[MAXDIM];
            static point fp[MAXDIM];
            int i;

            fa[depth] = pc->site_numm(t->key);
            fp[depth] = t->key;

            if (depth == p_fg_x_depth) for (i = 0; i <= depth; i++)
                {
                    fprintf(FG_OUT, "%d%s", fa[i], (i == depth) ? "\n" : " ");
                }
        }

        void PowerCrustHull::print_fg_alt(fg* faces_gr, FILE* F, int fd)
        {
            FG_OUT = F;

            if (!faces_gr)
            {
                return;
            }

            p_fg_x_depth = fd;
            visit_fg(faces_gr, &GraspStudio::PowerCrust::PowerCrustHull::p_fg_x);
            fclose(FG_OUT);
        }


        void PowerCrustHull::print_fg(fg* faces_gr, FILE* F)
        {
            FG_OUT = F;
            visit_fg(faces_gr, &GraspStudio::PowerCrust::PowerCrustHull::p_fg);
        }


        double fg_hist[100][100], fg_hist_bad[100][100], fg_hist_far[100][100];

        void PowerCrustHull::h_fg(Tree* t, int depth, int bad)
        {
            if (!t->fgs->facets)
            {
                return;
            }

            if (bad)
            {
                fg_hist_bad[depth][t->fgs->facets->size]++;
                return;
            }

            fg_hist[depth][t->fgs->facets->size]++;
        }

        void PowerCrustHull::h_fg_far(Tree* t, int depth)
        {
            if (t->fgs->facets)
            {
                fg_hist_far[depth][t->fgs->facets->size]++;
            }
        }


        void PowerCrustHull::print_hist_fg(simplex* root, fg* faces_gr, FILE* F)
        {
            int i, j, k;
            double tot_good[100], tot_bad[100], tot_far[100];

            for (i = 0; i < 20; i++)
            {
                tot_good[i] = tot_bad[i] = tot_far[i] = 0;

                for (j = 0; j < 100; j++)
                {
                    fg_hist[i][j] = fg_hist_bad[i][j] = fg_hist_far[i][j] = 0;
                }
            }

            if (!root)
            {
                return;
            }

            find_alpha(root);

            if (!faces_gr)
            {
                faces_gr = build_fg(root);
            }

            visit_fg(faces_gr, &GraspStudio::PowerCrust::PowerCrustHull::h_fg);
            visit_fg_far(faces_gr, &GraspStudio::PowerCrust::PowerCrustHull::h_fg_far);

            for (j = 0; j < 100; j++) for (i = 0; i < 20; i++)
                {
                    tot_good[i] += fg_hist[i][j];
                    tot_bad[i] += fg_hist_bad[i][j];
                    tot_far[i]  += fg_hist_far[i][j];
                }

            for (i = 19; i >= 0 && !tot_good[i] && !tot_bad[i]; i--);

            fprintf(F, "totals   ");

            for (k = 0; k <= i; k++)
            {
                if (k == 0)
                {
                    fprintf(F, "  ");
                }
                else
                {
                    fprintf(F, "            ");
                }

                fprintf(F, "%d/%d/%d",
                        (int)tot_far[k], (int)tot_good[k], (int)tot_good[k] + (int)tot_bad[k]);
            }


            for (j = 0; j < 100; j++)
            {
                for (i = 19; i >= 0 && !fg_hist[i][j] && !fg_hist_bad[i][j]; i--);

                if (i == -1)
                {
                    continue;
                }

                fprintf(F, "\n%d    ", j);
                fflush(F);

                for (k = 0; k <= i; k++)
                {
                    if (k == 0)
                    {
                        fprintf(F, "  ");
                    }
                    else
                    {
                        fprintf(F, "            ");
                    }

                    if (fg_hist[k][j] || fg_hist_bad[k][j])
                        fprintf(F,
                                "%2.1f/%2.1f/%2.1f",
                                tot_far[k] ? 100 * fg_hist_far[k][j] / tot_far[k] + .05 : 0,
                                tot_good[k] ? 100 * fg_hist[k][j] / tot_good[k] + .05 : 0,
                                100 * (fg_hist[k][j] + fg_hist_bad[k][j]) / (tot_good[k] + tot_bad[k]) + .05
                               );
                }
            }

            fprintf(F, "\n");
        }

        //////////////////////////////////////////////////////////////////////////




#ifdef WIN32
#define random rand
#define srandom srand
#endif


        PowerCrustHull::PowerCrustHull(PowerCrustPtr pc)
        {
            this->pc = pc;
            DFILE = pc->DFILE;
            heap_size = 0;

            memset(infinity, 0, sizeof(Coord) * 10);
            infinity[0] = 57.2; /* point at infinity for vd; value not used */
            memset(A, 0, sizeof(int) * 100);
            memset(B, 0, sizeof(int) * 100);
            memset(C, 0, sizeof(int) * 100);
            memset(D, 0, sizeof(int) * 100);
            tot = 0;
            totinf = 0;
            bigt = 0;
            crust_loopStart = -1;
            crust_count = 0;
            crust_lastCount = 0;
            crust_FALSE_i = 0;
            crust_TRUE_i = 1;
            power_numvtxs = 0;
            power_numfaces = 0;
        }




        //Coord infinity[10]={57.2,0,0,0,0}; /* point at infinity for vd; value not used */

        long pnum;

        int scount = 0; /* new power */
        POWERCRUST_STORAGE(simplex)

        //#define push(x) *(st+tms++) = x;
        //#define pop(x)  x = *(st + --tms);

        void* PowerCrustHull::visit_triang_gen(simplex* s, visit_func visit, test_func test)
        {
            /*
                 * starting at s, visit simplices t such that test(s,i,0) is true,
                 * and t is the i'th neighbor of s;
                 * apply visit function to all visited simplices;
                 * when visit returns nonNULL, exit and return its value
                 */
            neighbor* sn;
            void* v;
            simplex* t;
            int i;
            long tms = 0;
            static long vnum = -1;
            static long ss = 2000;
            static simplex** st;

            vnum--;

            if (!st)
            {
                assert(st = (simplex**)malloc((ss + MAXDIM + 1) * sizeof(simplex*)));
            }

            if (s)
            {
                *(st + tms++) = s;    // push(s);
            }

            while (tms)
            {

                if (tms > ss)
                {
                    //DEBEXP(-1,tms);
                    assert(st = (simplex**)realloc(st,
                                                   ((ss += ss) + MAXDIM + 1) * sizeof(simplex*)));
                }

                t = *(st + --tms);//pop(t);

                if (!t || t->visit == vnum)
                {
                    continue;
                }

                t->visit = vnum;

                if ((v = (*this.*visit)(t, 0)) != NULL)
                {
                    return v;
                }

                for (i = -1, sn = t->neigh - 1; i < cdim; i++, sn++)
                    if ((sn->simp->visit != vnum) && sn->simp && (*this.*test)(t, i, 0))
                    {
                        *(st + tms++) = sn->simp;    //push(sn->simp);
                    }
            }

            return NULL;
        }


        int PowerCrustHull::truet(simplex* s, int i, void* dum)
        {
            return 1;
        }
        int PowerCrustHull::hullt(simplex* s, int i, void* dummy)
        {
            return i > -1;
        }
        void* PowerCrustHull::facet_test(simplex* s, void* dummy)
        {
            return (!s->peak.vert) ? s : NULL;
        }


        void* PowerCrustHull::visit_triang(simplex* root, visit_func visit)
        /* visit the whole triangulation */
        {
            return visit_triang_gen(root, visit, &GraspStudio::PowerCrust::PowerCrustHull::truet);
        }

        void* PowerCrustHull::visit_hull(simplex* root, visit_func visit)
        /* visit all simplices with facets of the current hull */
        {
            return visit_triang_gen((simplex*) visit_triang(root, &GraspStudio::PowerCrust::PowerCrustHull::facet_test) , visit, &GraspStudio::PowerCrust::PowerCrustHull::hullt);
        }



#define lookup(a,b,what,whatt)                      \
    {                                   \
        int i;                              \
        neighbor *x;                            \
        for (i=0, x = a->neigh; (x->what != b) && (i<cdim) ; i++, x++); \
        if (i<cdim)                         \
            return x;                       \
        else {                              \
            fprintf(DFILE,"adjacency failure,op_" #what ":\n"); \
            print_simplex_f(a, DFILE, &GraspStudio::PowerCrust::PowerCrustHull::print_neighbor_full);    \
            print_##whatt(b, DFILE);                \
            fprintf(DFILE,"---------------------\n");       \
            print_triang(a,DFILE, &GraspStudio::PowerCrust::PowerCrustHull::print_neighbor_full);        \
            exit(1);                        \
            return 0;                       \
        }                               \
    }                                   \
     

        neighbor* PowerCrustHull::op_simp(simplex* a, simplex* b)
        {
            lookup(a, b, simp, simplex)
        }
        /* the neighbor entry of a containing b */

        neighbor* PowerCrustHull::op_vert(simplex* a, site b)
        {
            lookup(a, b, vert, site)
        }
        /* the neighbor entry of a containing b */


        void PowerCrustHull::connect(simplex* s)
        {
            /* make neighbor connections between newly created simplices incident to p */

            site xf, xb, xfi;
            simplex* sb, *sf, *seen;
            int i;
            neighbor* sn;
#ifdef HULL_DEBUG_TO_FILE
            logHull.log("connect:");
            logHull.log(pnum);

            if (s)
            {
                logHull.log("s");
            }
            else
            {
                logHull.log("no_s");
            }

#endif // HULL_DEBUG_TO_FILE

            if (!s)
            {
                return;
            }

            assert(!s->peak.vert
                   && s->peak.simp->peak.vert == pc->p
                   && !op_vert(s, pc->p)->simp->peak.vert);
#ifdef HULL_DEBUG_TO_FILE
            logHull.log(s->visit);
#endif

            if (s->visit == pnum)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("_1_");
#endif
                return;
            }

            s->visit = pnum;
            seen = s->peak.simp;
            xfi = op_simp(seen, s)->vert;
#ifdef HULL_DEBUG_TO_FILE

            if (xfi)
            {
                logHull.log(xfi[0]);
                logHull.log(xfi[1]);
                logHull.log(xfi[2]);
            }
            else
            {
                logHull.log("no_x");
            }

#endif

            for (i = 0, sn = s->neigh; i < cdim; i++, sn++)
            {
                xb = sn->vert;
#ifdef HULL_DEBUG_TO_FILE

                if (xb)
                {
                    logHull.log(xb[0]);
                    logHull.log(xb[1]);
                    logHull.log(xb[2]);
                }
                else
                {
                    logHull.log("no_xb");
                }

#endif

                if (pc->p == xb)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("_2_");
#endif
                    continue;
                }

                sb = seen;
                sf = sn->simp;
                xf = xfi;

                if (!sf->peak.vert)     /* are we done already? */
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("_3_");
#endif
                    sf = op_vert(seen, xb)->simp;

                    if (sf->peak.vert)
                    {
#ifdef HULL_DEBUG_TO_FILE
                        logHull.log("_4_");
#endif
                        continue;
                    }
                }
                else do
                    {
                        xb = xf;
                        xf = op_simp(sf, sb)->vert;
                        sb = sf;
                        sf = op_vert(sb, xb)->simp;
#ifdef HULL_DEBUG_TO_FILE
                        logHull.log("_5_");

                        if (xf)
                        {
                            logHull.log(xf[0]);
                            logHull.log(xf[1]);
                            logHull.log(xf[2]);
                        }
                        else
                        {
                            logHull.log("no_xf");
                        }

#endif
                    }
                    while (sf->peak.vert);

                sn->simp = sf;
                op_vert(sf, xf)->simp = s;

                connect(sf);
            }

        }



        simplex* PowerCrustHull::make_facets(simplex* seen)
        {
            /*
             * visit simplices s with sees(p,s), and make a facet for every neighbor
             * of s not seen by pc->p
             */

            simplex* n;
            static simplex* ns;
            neighbor* bn;
            int i;


#ifdef HULL_DEBUG_TO_FILE
            logHull.log("make_facets:");

            if (seen)
            {
                logHull.log("simplex");
            }
            else
            {
                logHull.log("null simplex");
            }

#endif // HULL_DEBUG_TO_FILE

            if (!seen)
            {
                return NULL;
            }

            // bool ok = (sees(pc->p,seen) && !seen->peak.vert);
            //GRASPSTUDIO_ASSERT(ok);

            seen->peak.vert = pc->p;

            for (i = 0, bn = seen->neigh; i < cdim; i++, bn++)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log(i);
#endif // HULL_DEBUG_TO_FILE
                n = bn->simp;

                if (pnum != n->visit)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("_1_");
#endif // HULL_DEBUG_TO_FILE
                    n->visit = pnum;

                    if (sees(pc->p, n))
                    {
#ifdef HULL_DEBUG_TO_FILE
                        logHull.log("_2_");
#endif // HULL_DEBUG_TO_FILE
                        make_facets(n);
                    }
                    else
                    {
#ifdef HULL_DEBUG_TO_FILE
                        logHull.log("_3_");
#endif // HULL_DEBUG_TO_FILE
                    }
                }

                if (n->peak.vert)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("_4_");
#endif // HULL_DEBUG_TO_FILE
                    continue;
                }

                POWERCRUST_copy_simp(ns, seen);
                ns->visit = 0;
                ns->peak.vert = 0;
                ns->normal = 0;
                ns->peak.simp = seen;
                /*      ns->Sb -= ns->neigh[i].basis->sqb; */
                POWERCRUST_NULLIFY(basis_s, ns->neigh[i].basis);
                ns->neigh[i].vert = pc->p;
                bn->simp = op_simp(n, seen)->simp = ns;
#ifdef HULL_DEBUG_TO_FILE
                logHull.log(bn->simp->sqradius);
                logHull.log(bn->simp->poleindex);
                logHull.log(bn->simp->visit);
#endif // HULL_DEBUG_TO_FILE
            }

            return ns;
        }



        simplex* PowerCrustHull::extend_simplices(simplex* s)
        {
            /*
             * pc->p lies outside flat containing previous sites;
             * make pc->p a vertex of every current simplex, and create some new simplices
             */
#ifdef HULL_DEBUG_TO_FILE
            logHull.logString("extend_simplices:", "", false);
            logHull.logLong(pnum, ",", false);

            if (s)
            {
                logHull.logLong(s->visit, ",", false);
            }
            else
            {
                logHull.log("null simplex");
            }

#endif // HULL_DEBUG_TO_FILE

            int i,
                ocdim = cdim - 1;
            simplex* ns;
            neighbor* nsn;

            if (s->visit == pnum)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.logString("_1_", "", false);
#endif // HULL_DEBUG_TO_FILE
                simplex* ret;

                if (s->peak.vert)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.logString("_2_");
#endif // HULL_DEBUG_TO_FILE
                    ret = s->neigh[ocdim].simp;
                }
                else
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.logString("_3_");
#endif // HULL_DEBUG_TO_FILE
                    ret = s;
                }

                return ret;
            }

            s->visit = pnum;
            s->neigh[ocdim].vert = pc->p;

#ifdef HULL_DEBUG_TO_FILE
            logHull.logInt(ocdim, ",", false);
            logHull.logDouble(s->neigh[ocdim].vert[0], ",", false);
            logHull.logDouble(s->neigh[ocdim].vert[1], ",", false);
            logHull.logDouble(s->neigh[ocdim].vert[2], ",", false);
#endif // HULL_DEBUG_TO_FILE

            POWERCRUST_NULLIFY(basis_s, s->normal);
            POWERCRUST_NULLIFY(basis_s, s->neigh[0].basis);

            if (!s->peak.vert)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("_4_");

                if (s->peak.simp->vv)
                {
                    logHull.log(s->peak.simp->vv[0]);
                    logHull.log(s->peak.simp->vv[1]);
                    logHull.log(s->peak.simp->vv[2]);
                }

#endif // HULL_DEBUG_TO_FILE
                s->neigh[ocdim].simp = extend_simplices(s->peak.simp);
#ifdef HULL_DEBUG_TO_FILE

                if (s->neigh[ocdim].simp->vv)
                {
                    logHull.log(s->neigh[ocdim].simp->vv[0]);
                    logHull.log(s->neigh[ocdim].simp->vv[1]);
                    logHull.log(s->neigh[ocdim].simp->vv[2]);
                }

#endif // HULL_DEBUG_TO_FILE
                return s;
            }
            else
            {
                POWERCRUST_copy_simp(ns, s);
                s->neigh[ocdim].simp = ns;
                ns->peak.vert = NULL;
                ns->peak.simp = s;
                ns->neigh[ocdim] = s->peak;
                POWERCRUST_inc_ref(basis_s, s->peak.basis);

                for (i = 0, nsn = ns->neigh; i < cdim; i++, nsn++)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log(i);

                    if (nsn->simp && nsn->simp->vv)
                    {
                        logHull.log(nsn->simp->vv[0]);
                        logHull.log(nsn->simp->vv[1]);
                        logHull.log(nsn->simp->vv[2]);
                    }
                    else
                    {
                        logHull.log("no simp");
                    }

#endif // HULL_DEBUG_TO_FILE
                    nsn->simp = extend_simplices(nsn->simp);
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log(i);

                    if (nsn->simp->vv)
                    {
                        logHull.log(nsn->simp->vv[0]);
                        logHull.log(nsn->simp->vv[1]);
                        logHull.log(nsn->simp->vv[2]);
                    }
                    else
                    {
                        logHull.log("no simp2");
                    }

#endif // HULL_DEBUG_TO_FILE
                }
            }

            return ns;
        }


        simplex* PowerCrustHull::search(simplex* root)
        {
            /* return a simplex s that corresponds to a facet of the
             * current hull, and sees(pc->p, s) */
#ifdef HULL_DEBUG_TO_FILE
            logHull.log("search:");
#endif
            simplex* s;
            static simplex** st;
            static long ss = MAXDIM;
            neighbor* sn;
            int i;
            long tms = 0;

            if (!st)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("!st");
#endif
                st = (simplex**)malloc((ss + MAXDIM + 1) * sizeof(simplex*));
            }
            else
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("st");
#endif
            }

            *(st + tms++) = root->peak.simp; //push(root->peak.simp);
            root->visit = pnum;
#ifdef HULL_DEBUG_TO_FILE
            logHull.log(pnum);
#endif

            if (!sees(pc->p, root))
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("#1#");
#endif

                for (i = 0, sn = root->neigh; i < cdim; i++, sn++)
                {
                    *(st + tms++) = sn->simp;    //push(sn->simp);
                }
            }

            while (tms)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log(tms);
                logHull.log(ss);
#endif

                if (tms > ss)
                    assert(st = (simplex**)realloc(st,
                                                   ((ss += ss) + MAXDIM + 1) * sizeof(simplex*)));

                s = *(st + --tms);//pop(s);
#ifdef HULL_DEBUG_TO_FILE
                logHull.log(s->visit);
#endif

                if (s->visit == pnum)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("#2#");
#endif
                    continue;
                }

                s->visit = pnum;

                if (!sees(pc->p, s))
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("#3#");
#endif
                    continue;
                }

                if (!s->peak.vert)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.logNewLine("#4#");
#endif
                    return s;
                }

                for (i = 0, sn = s->neigh; i < cdim; i++, sn++)
                {
                    *(st + tms++) = sn->simp;    //push(sn->simp);
                }
            }

#ifdef HULL_DEBUG_TO_FILE
            logHull.logNewLine("#5#");
#endif
            return NULL;
        }


        point PowerCrustHull::get_another_site(void)
        {

            /*static int scount =0; */
            point pnext;

            if (!(++scount % 1000))
            {
                fprintf(DFILE, "site %d...", scount);
            }

            /*  check_triang(); */
            pnext = (pc->get_next_site());

            if (!pnext)
            {
                return NULL;
            }

            pnum = pc->site_numm(pnext) + 2;
            return pnext;
        }



        void PowerCrustHull::buildhull(simplex* root)
        {
#ifdef HULL_DEBUG_TO_FILE
            logHull.logString("--  buildHull --");
#endif

            while (cdim < rdim)
            {
                pc->p = get_another_site();

                if (!pc->p)
                {
                    return;
                }

#ifdef HULL_DEBUG_TO_FILE
                logHull.logDouble(pc->p[0], ",", false);
                logHull.logDouble(pc->p[1], ",", false);
                logHull.logDouble(pc->p[2], ",", false);
#endif

                if (out_of_flat(root, pc->p))
                {
                    extend_simplices(root);
                }
                else
                {
                    connect(make_facets(search(root)));
                }
            }

            int dbgCount = 0;
            pc->p = get_another_site();

            while ((pc->p) != NULL)
            {
                dbgCount++;
                //printf ("dbgCount:%i\n",dbgCount);
#ifdef HULL_DEBUG_TO_FILE
                logHull.logNewLine();
                logHull.logString(" ############ loop", "", false);
                logHull.logInt(dbgCount, "", false);
                logHull.logString(" ############");
#endif // HULL_DEBUG_TO_FILE

                connect(make_facets(search(root)));
                pc->p = get_another_site();
            }
        }


        /* for each pole array, compute the maximum of the distances on the sample */


        void PowerCrustHull::compute_distance(simplex** poles, int size, double* distance)
        {

            int i, j, k, l;
            double indices[4][3]; /* the coords of the four vertices of the simplex*/
            point v[MAXDIM];
            simplex* currSimplex;



            double maxdistance = 0;
            double currdistance;

            for (l = 0; l < size; l++) /* for each pole do*/
            {

                if (poles[l] != NULL)
                {
                    currSimplex = poles[l];




                    /* get the coordinates of the  four endpoints */
                    for (j = 0; j < 4; j++)
                    {
                        v[j] = currSimplex->neigh[j].vert;

                        for (k = 0; k < 3; k++)
                        {
                            indices[j][k] = v[j][k] / pc->mult_up;
                        }


                    }

                    /* now compute the actual distance  */
                    maxdistance = 0;

                    for (i = 0; i < 4; i++)
                    {
                        for (j = i + 1; j < 4; j++)
                        {
                            currdistance = SQ(indices[i][0] - indices[j][0]) +
                                           SQ(indices[i][1] - indices[j][1]) + SQ(indices[i][2] - indices[j][2]);
                            currdistance = sqrt(currdistance);

                            if (maxdistance < currdistance)
                            {
                                maxdistance = currdistance;
                            }
                        }
                    }

                    distance[l] = maxdistance;

                }

            }
        }















        //#define PRINT_SQVALS

#define DEBS(qq)  {if (DEBUG>qq) {
#define EDEBS }}
#define DEBOUT DFILE
#define DEB(ll,mes)  DEBS(ll) fprintf(DEBOUT,#mes "\n");fflush(DEBOUT); EDEBS
#define DEBEXP(ll,exp) DEBS(ll) fprintf(DEBOUT,#exp "=%G\n", (double) exp); fflush(DEBOUT); EDEBS
#define DEBTR(ll) DEBS(ll) fprintf(DEBOUT, __FILE__ " line %d \n" ,__LINE__);fflush(DEBOUT); EDEBS




        short check_overshoot_f = 0;


        simplex* ch_root;

#define NEARZERO(d) ((d) < FLT_EPSILON && (d) > -FLT_EPSILON)
#define SMALL (100*FLT_EPSILON)*(100*FLT_EPSILON)

#define SWAP(X,a,b) {X t; t = a; a = b; b = t;}

#define DMAX

#define check_overshoot(x)                          \
    {if (CHECK_OVERSHOOT && check_overshoot_f && ((x)>9e15))        \
            VR_WARNING << "overshot exact arithmetic" << endl;}            \
     

#define DELIFT 0
        int basis_vec_size;


        int exact_bits;
        float b_err_min, b_err_min_sq;

        double logb(double); /* on SGI machines: returns floor of log base 2 */

        static short vd;
        static basis_s  tt_basis = {0, 1, -1, 0, 0, {0}};
        static basis_s* tt_basisp = &tt_basis, *infinity_basis;


        POWERCRUST_STORAGE(basis_s)

        typedef Coord site_struct;
#define VA(x) ((x)->vecs+rdim)
#define VB(x) ((x)->vecs)



#define two_to(x) ( ((x)<20) ? 1<<(x) : ldexp(1.0,(x)) )




#define lookupshort(a,b,whatb,c,whatc)                  \
    {                                   \
        int i;                              \
        neighbor *x;                            \
        c = NULL;                           \
        for (i=-1, x = a->neigh-1; (x->whatb != b) && (i<cdim) ; i++, x++);\
        if (i<cdim) c = x->whatc;                   \
    }                                   \
     

        Coord PowerCrustHull::Vec_dot(point x, point y)
        {
            int i;
            Coord sum = 0;

            for (i = 0; i < rdim; i++)
            {
                sum += x[i] * y[i];
            }

            return sum;
        }

        Coord PowerCrustHull::Vec_dot_pdim(point x, point y)
        {
            int i;
            Coord sum = 0;

            for (i = 0; i < pdim; i++)
            {
                sum += x[i] * y[i];
            }

            /*  check_overshoot(sum); */
            return sum;
        }

        Coord PowerCrustHull::Norm2(point x)
        {
            int i;
            Coord sum = 0;

            for (i = 0; i < rdim; i++)
            {
                sum += x[i] * x[i];
            }

            return sum;
        }

        void PowerCrustHull::Ax_plus_y(Coord a, point x, point y)
        {
            int i;

            for (i = 0; i < rdim; i++)
            {
                *y++ += a** x++;
            }
        }

        void PowerCrustHull::Ax_plus_y_test(Coord a, point x, point y)
        {
            int i;

            for (i = 0; i < rdim; i++)
            {
                check_overshoot(*y + a** x);
                *y++ += a** x++;
            }
        }

        void PowerCrustHull::Vec_scale(int n, Coord a, Coord* x)
        {
            register Coord* xx = x,
                            *xend = xx + n;

            while (xx != xend)
            {
                *xx++ *= a;
            }
        }

        void PowerCrustHull::Vec_scale_test(int n, Coord a, Coord* x)
        {
            register Coord* xx = x,
                            *xend = xx + n  ;

            check_overshoot(a);

            while (xx != xend)
            {
                *xx *= a;
                check_overshoot(*xx);
                xx++;
            }
        }




        void PowerCrustHull::print_site(site p, FILE* F)
        {
            print_point(F, pdim, p);
            fprintf(F, "\n");
        }


        double PowerCrustHull::sc(basis_s* v, simplex* s, int k, int j)
        {
            /* amount by which to scale up vector, for reduce_inner */

            double      labound;
            static int  lscale;
            static double   max_scale,
                   ldetbound,
                   Sb;

            if (j < 10)
            {
                labound = logb(v->sqa) / 2;
                max_scale = exact_bits - labound - 0.66 * (k - 2) - 1  - DELIFT;

                if (max_scale < 1)
                {
                    VR_WARNING << "overshot exact arithmetic" << endl;    //MP2013-08-29 (temporarily) removed output
                    max_scale = 1;

                }

                if (j == 0)
                {
                    int i;
                    neighbor* sni;
                    basis_s* snib;

                    ldetbound = DELIFT;

                    Sb = 0;

                    for (i = k - 1, sni = s->neigh + k - 1; i > 0; i--, sni--)
                    {
                        snib = sni->basis;
                        Sb += snib->sqb;
                        ldetbound += logb(snib->sqb) / 2 + 1;
                        ldetbound -= snib->lscale;
                    }
                }
            }

            if (ldetbound - v->lscale + logb(v->sqb) / 2 + 1 < 0)
            {
                /*DEBS(-2)
                    DEBTR(-2) DEBEXP(-2, ldetbound)
                    print_simplex_f(s, DFILE, &print_neighbor_full);
                print_basis(DFILE,v);
                EDEBS*/
                return 0;
            }
            else
            {
                lscale = (int)(logb(2 * Sb / (v->sqb + v->sqa * b_err_min)) / 2);

                if (lscale > max_scale)
                {
                    lscale = (int)max_scale;
                }
                else if (lscale < 0)
                {
                    lscale = 0;
                }

                v->lscale += lscale;
                return two_to(lscale);
            }
        }


        double PowerCrustHull::lower_terms(basis_s* v)
        {

            point vp = v->vecs;
            int i, j, h, hh = 0;
            int facs[6] = {2, 3, 5, 7, 11, 13};
            double out = 1;

            DEBTR(-10) print_basis(DFILE, v);
            printf("\n");
            DEBTR(0)

            for (j = 0; j < 6; j++) do
                {
                    for (i = 0; i < 2 * rdim && facs[j]*floor(vp[i] / facs[j]) == vp[i]; i++);

                    if ((h = (i == 2 * rdim)) != 0)
                    {
                        hh = 1;
                        out *= facs[j];

                        for (i = 0; i < 2 * rdim; i++)
                        {
                            vp[i] /= facs[j];
                        }
                    }
                }
                while (h);

            /*  if (hh) {DEBTR(-10)  print_basis(DFILE, v);} */
            return out;
        }

        double PowerCrustHull::lower_terms_point(point vp)
        {

            int i, j, h, hh = 0;
            int facs[6] = {2, 3, 5, 7, 11, 13};
            double out = 1;

            for (j = 0; j < 6; j++) do
                {
                    for (i = 0; i < 2 * rdim && facs[j]*floor(vp[i] / facs[j]) == vp[i]; i++);

                    if ((h = (i == 2 * rdim)) != 0)
                    {
                        hh = 1;
                        out *= facs[j];

                        for (i = 0; i < 2 * rdim; i++)
                        {
                            vp[i] /= facs[j];
                        }
                    }
                }
                while (h);

            return out;
        }


        int PowerCrustHull::reduce_inner(basis_s* v, simplex* s, int k)
        {

            point   va = VA(v),
                    vb = VB(v);
            int i, j;
            double  dd;
            double  scale;
            basis_s* snibv;
            neighbor* sni;
            static int failcount;

            /*  lower_terms(v); */
            v->sqa = v->sqb = Norm2(vb);
#ifdef PRINT_SQVALS
            printf("a:%f\n", v->sqa);
#endif
#ifdef HULL_DEBUG_TO_FILE
            logHull.logString("reduce_inner:", "", false);
            logHull.logInt(failcount, "", false);
#endif

            if (k <= 1)
            {
                memcpy(vb, va, basis_vec_size);
#ifdef HULL_DEBUG_TO_FILE
                logHull.logString("return 1");
#endif
                return 1;
            }

            /*  if (vd) {
                snibv = s->neigh[1].basis;
                scale = floor(sqrt(snibv->sqa/v->sqa));
                if (scale > 1) Vec_scale(rdim,scale,va);
                dd = Vec_dot(VA(snibv),va)/snibv->sqa;
                dd = -floor(0.5+dd);
                Ax_plus_y( dd, VA(snibv), va);
                }
            */
            for (j = 0; j < 250; j++)
            {

                memcpy(vb, va, basis_vec_size);

                for (i = k - 1, sni = s->neigh + k - 1; i > 0; i--, sni--)
                {
                    snibv = sni->basis;
                    dd = -Vec_dot(VB(snibv), vb) / snibv->sqb;
                    Ax_plus_y(dd, VA(snibv), vb);
                }

                v->sqb = Norm2(vb);
                v->sqa = Norm2(va);
#ifdef HULL_DEBUG_TO_FILE
                logHull.logDouble((v->sqb), ",", false);
                logHull.logDouble((v->sqa), ",", false);
#endif
#ifdef PRINT_SQVALS
                printf("a:%f,\tb:%f\n", v->sqa, v->sqb);
#endif

                if (2 * v->sqb >= v->sqa)
                {
                    B[j]++;
                    return 1;
                }

                Vec_scale_test(rdim, scale = sc(v, s, k, j), va);

                for (i = k - 1, sni = s->neigh + k - 1; i > 0; i--, sni--)
                {
                    snibv = sni->basis;
                    dd = -Vec_dot(VB(snibv), va) / snibv->sqb;
                    dd = floor(dd + 0.5);
                    Ax_plus_y_test(dd, VA(snibv), va);
                }
            }

            if (failcount++ < 10)
            {
                //DEB(-8, reduce_inner failed on:)
                //  DEBTR(-8) print_basis(DFILE, v);
                print_simplex_f(s, DFILE, &GraspStudio::PowerCrust::PowerCrustHull::print_neighbor_full);
            }

#ifdef HULL_DEBUG_TO_FILE
            logHull.logNewLine();
#endif
            return 0;
        }

#define trans(z,p,q) {int i; for (i=0;i<pdim;i++) z[i+rdim] = z[i] = p[i] - q[i];}
#define lift(z,s) {if (vd) z[2*rdim-1] =z[rdim-1]= ldexp(Vec_dot_pdim(z,z), -DELIFT);}
        /*not scaling lift to 2^-DELIFT */



        int PowerCrustHull::reduce(basis_s** v, point p, simplex* s, int k)
        {

            point   z;
            point   tt = s->neigh[0].vert;

            if (!*v) POWERCRUST_NEWLRC(basis_s, (*v))
                else
                {
                    (*v)->lscale = 0;
                }

            z = VB(*v);

            if (vd || pc->power_diagram)
            {
                if (p == infinity)
                {
                    memcpy(*v, infinity_basis, basis_s_size);
#ifdef HULL_DEBUG_TO_FILE
                    logHull.logString("infscale:", "", false);
                    logHull.logInt(infinity_basis->lscale);
#endif
                }
                else
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.logString("_tr_lift_", "", false);
#endif
                    trans(z, p, tt);
                    lift(z, s);
                }
            }
            else
            {
                trans(z, p, tt);
#ifdef HULL_DEBUG_TO_FILE
                logHull.logString("_tr_", "", false);
#endif
            }

#ifdef HULL_DEBUG_TO_FILE
            logHull.logString("reduce:", "", false);
            logHull.logInt((*v)->lscale);
            logHull.logDouble(z[0], ",", false);
            logHull.logDouble(z[1], ",", false);
            logHull.logDouble(z[2]);
            logHull.logDouble(tt[0], ",", false);
            logHull.logDouble(tt[1], ",", false);
            logHull.logDouble(tt[2]);
            logHull.logDouble(p[0], ",", false);
            logHull.logDouble(p[1], ",", false);
            logHull.logDouble(p[2]);
#endif
            return reduce_inner(*v, s, k);
        }


        void PowerCrustHull::get_basis_sede(simplex* s)
        {

            int k = 1;
            neighbor* sn = s->neigh + 1,
                      *sn0 = s->neigh;
#ifdef HULL_DEBUG_TO_FILE
            logHull.logString("get_basis_sede:", "", false);

            if (sn && sn->vert)
            {
                logHull.logDouble(sn->vert[0], ",", false);
                logHull.logDouble(sn->vert[1], ",", false);
                logHull.logDouble(sn->vert[2]);
            }
            else
            {
                logHull.logString("**NO SN**");
            }

            if (sn0 && sn0->vert)
            {
                logHull.logDouble(sn0->vert[0], ",", false);
                logHull.logDouble(sn0->vert[1], ",", false);
                logHull.logDouble(sn0->vert[2]);
            }
            else
            {
                logHull.logString("**NO SN0**");
            }

#endif

            if ((vd || pc->power_diagram) && sn0->vert == infinity && cdim > 1)
            {
                SWAP(neighbor, *sn0, *sn);
                POWERCRUST_NULLIFY(basis_s, sn0->basis);
                sn0->basis = tt_basisp;
                tt_basisp->ref_count++;
#ifdef HULL_DEBUG_TO_FILE
                logHull.logString("_1_");
#endif
            }
            else
            {
                if (!sn0->basis)
                {
                    sn0->basis = tt_basisp;
                    tt_basisp->ref_count++;
#ifdef HULL_DEBUG_TO_FILE
                    logHull.logString("_2_");
#endif
                }
                else
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.logString("_3_");
                    logHull.logInt(k);
                    logHull.logInt(cdim);
#endif

                    while (k < cdim && sn->basis)
                    {
                        k++;
                        sn++;
                    }
                }
            }

            while (k < cdim)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.logString("_4_");
                logHull.logInt(k);
                logHull.logInt(cdim);
#endif
                POWERCRUST_NULLIFY(basis_s, sn->basis);
                reduce(&sn->basis, sn->vert, s, k);
                k++;
                sn++;
            }
        }


        int PowerCrustHull::out_of_flat(simplex* root, point p)
        {

            static neighbor p_neigh = {0, 0, 0};

            if (!p_neigh.basis)
            {
                p_neigh.basis = (basis_s*) malloc(basis_s_size);
            }

            p_neigh.vert = p;
            cdim++;
#ifdef HULL_DEBUG_TO_FILE
            logHull.logString("out_of_flat:", "", false);
            logHull.logInt(cdim, ",", false);
            logHull.logDouble(p[0], ",", false);
            logHull.logDouble(p[1], ",", false);
            logHull.logDouble(p[2], ",", false);
            logHull.logNewLine();
#endif
            root->neigh[cdim - 1].vert = root->peak.vert;
            POWERCRUST_NULLIFY(basis_s, root->neigh[cdim - 1].basis);
            get_basis_sede(root);

            if ((vd || pc->power_diagram) && root->neigh[0].vert == infinity)
            {
                return 1;
            }

            reduce(&p_neigh.basis, p, root, cdim);

            if (p_neigh.basis->sqa != 0)
            {
                return 1;
            }

            cdim--;
            return 0;
        }


        double PowerCrustHull::cosangle_sq(basis_s* v, basis_s* w)
        {
            double dd;
            point   vv = v->vecs,
                    wv = w->vecs;
            dd = Vec_dot(vv, wv);
            return dd * dd / Norm2(vv) / Norm2(wv);
        }


        int PowerCrustHull::check_perps(simplex* s)
        {

            static basis_s* b = NULL;
            point   z, y;
            point   tt;
            //double  dd;
            int i, j;

            for (i = 1; i < cdim; i++) if (NEARZERO(s->neigh[i].basis->sqb))
                {
                    return 0;
                }

            if (!b)
            {
                assert(b = (basis_s*)malloc(basis_s_size));
            }
            else
            {
                b->lscale = 0;
            }

            z = VB(b);
            tt = s->neigh[0].vert;

            for (i = 1; i < cdim; i++)
            {
                y = s->neigh[i].vert;

                if ((vd || pc->power_diagram) && y == infinity)
                {
                    memcpy(b, infinity_basis, basis_s_size);
                }
                else
                {
                    trans(z, y, tt);
                    lift(z, s);
                }

                if (s->normal && cosangle_sq(b, s->normal) > b_err_min_sq)
                {
                    /*
                    DEBS(0)
                                                                             DEB(0,bad normal) DEBEXP(0,i) DEBEXP(0,dd)
                                                                             print_simplex_f(s, DFILE, &print_neighbor_full);
                    EDEBS*/
                    return 0;
                }

                for (j = i + 1; j < cdim; j++)
                {
                    if (cosangle_sq(b, s->neigh[j].basis) > b_err_min_sq)
                    {
                        /*DEBS(0)
                            DEB(0,bad basis)DEBEXP(0,i) DEBEXP(0,j) DEBEXP(0,dd)
                            DEBTR(-8) print_basis(DFILE, b);
                        print_simplex_f(s, DFILE, &print_neighbor_full);
                        EDEBS*/
                        return 0;
                    }
                }
            }

            return 1;
        }


        void PowerCrustHull::get_normal_sede(simplex* s)
        {

            neighbor* rn;
            int i, j;

            get_basis_sede(s);

            if (rdim == 3 && cdim == 3)
            {
                point   c,
                        a = VB(s->neigh[1].basis),
                        b = VB(s->neigh[2].basis);
                POWERCRUST_NEWLRC(basis_s, s->normal);
                c = VB(s->normal);
                c[0] = a[1] * b[2] - a[2] * b[1];
                c[1] = a[2] * b[0] - a[0] * b[2];
                c[2] = a[0] * b[1] - a[1] * b[0];
                s->normal->sqb = Norm2(c);

                for (i = cdim + 1, rn = ch_root->neigh + cdim - 1; i; i--, rn--)
                {
                    for (j = 0; j < cdim && rn->vert != s->neigh[j].vert; j++);

                    if (j < cdim)
                    {
                        continue;
                    }

                    if (rn->vert == infinity)
                    {
                        if (c[2] > -b_err_min)
                        {
                            continue;
                        }
                    }
                    else  if (!sees(rn->vert, s))
                    {
                        continue;
                    }

                    c[0] = -c[0];
                    c[1] = -c[1];
                    c[2] = -c[2];
                    break;
                }

                //DEBS(-1) if (!check_perps(s)) exit(1); EDEBS
                bool ok = check_perps(s) != 0;
                return;
            }

            for (i = cdim + 1, rn = ch_root->neigh + cdim - 1; i; i--, rn--)
            {
                for (j = 0; j < cdim && rn->vert != s->neigh[j].vert; j++);

                if (j < cdim)
                {
                    continue;
                }

                reduce(&s->normal, rn->vert, s, cdim);

                if (s->normal->sqb != 0)
                {
                    break;
                }
            }

            //DEBS(-1) if (!check_perps(s)) {DEBTR(-1) exit(1);} EDEBS
            bool ok = check_perps(s) != 0;

        }


        void PowerCrustHull::get_normal(simplex* s)
        {
            get_normal_sede(s);
            return;
        }

        int PowerCrustHull::sees(site p, simplex* s)
        {
#ifdef HULL_DEBUG_TO_FILE
            logHull.log("sees:");

            logHull.log(p[0]);
            logHull.log(p[1]);
            logHull.log(p[2]);
#endif
            static basis_s* b = NULL;
            point   tt, zz;
            double  dd, dds;
            int i;


            if (!b)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("!b");
#endif
                assert(b = (basis_s*)malloc(basis_s_size));
            }
            else
            {
                b->lscale = 0;
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("b");
#endif
            }

            zz = VB(b);
#ifdef HULL_DEBUG_TO_FILE
            logHull.log(zz[0]);
            logHull.log(zz[1]);
            logHull.log(zz[2]);
#endif

            if (cdim == 0)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("cdim=0");
#endif
                return 0;
            }

            if (!s->normal)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("!normal");
#endif
                get_normal_sede(s);

                for (i = 0; i < cdim; i++)
                {
                    POWERCRUST_NULLIFY(basis_s, s->neigh[i].basis);
                }
            }

            tt = s->neigh[0].vert;
#ifdef HULL_DEBUG_TO_FILE
            logHull.log(tt[0]);
            logHull.log(tt[1]);
            logHull.log(tt[2]);
#endif

            if (vd || pc->power_diagram)
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("_1_");
#endif

                if (p == infinity)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("_2_");
#endif
                    memcpy(b, infinity_basis, basis_s_size);
                }
                else
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log("_3_");
#endif
                    trans(zz, p, tt);
                    lift(zz, s);
                }
            }
            else
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.log("_4_");
#endif
                trans(zz, p, tt);
            }

            for (i = 0; i < 3; i++)
            {
                dd = Vec_dot(zz, s->normal->vecs);
#ifdef HULL_DEBUG_TO_FILE
                logHull.log(dd);
#endif

                if (dd == 0.0)
                {
                    //DEBS(-7) DEB(-6,degeneracy:); DEBEXP(-6,site_num(p));
                    //print_site(p, DFILE); print_simplex_f(s, DFILE, &print_neighbor_full); EDEBS
                    return 0;
                }

                dds = dd * dd / s->normal->sqb / Norm2(zz);
#ifdef HULL_DEBUG_TO_FILE
                logHull.log(dds);
#endif

                if (dds > b_err_min_sq)
                {
#ifdef HULL_DEBUG_TO_FILE
                    logHull.log(dd);
#endif
                    return (dd < 0);
                }

                get_basis_sede(s);
                reduce_inner(b, s, cdim);
            }

            /*
            DEBS(-7) if (i==3) {
                DEB(-6, looped too much in sees);
                DEBEXP(-6,dd) DEBEXP(-6,dds) DEBEXP(-6,site_num(p));
                print_simplex_f(s, DFILE, &print_neighbor_full); exit(1);}
            EDEBS
            */
            return 0;
        }





        double PowerCrustHull::radsq(simplex* s)
        {

            point n;
            neighbor* sn;
            int i;

            /* square of ratio of circumcircle radius to
               max edge length for Delaunay tetrahedra */


            for (i = 0, sn = s->neigh; i < cdim; i++, sn++)
                if (sn->vert == infinity)
                {
                    return Huge;
                }

            if (!s->normal)
            {
                get_normal_sede(s);
            }

            /* compute circumradius */
            n = s->normal->vecs;

            if (NEARZERO(n[rdim - 1]))
            {
                return Huge;
            }

            return Vec_dot_pdim(n, n) / 4 / n[rdim - 1] / n[rdim - 1];
        }


        void* PowerCrustHull::zero_marks(simplex* s, void* dum)
        {
            s->mark = 0;
            return NULL;
        }

        void* PowerCrustHull::one_marks(simplex* s, void* dum)
        {
            s->mark = 1;
            return NULL;
        }

        void* PowerCrustHull::show_marks(simplex* s, void* dum)
        {
            printf("%d", s->mark);
            return NULL;
        }


#define swap_points(a,b) {point t; t=a; a=b; b=t;}

        int PowerCrustHull::alph_test(simplex* s, int i, void* alphap)
        {
            /*returns 1 if not an alpha-facet */

            simplex* si;
            double  rs, rsi, rsfi;
            neighbor* scn, *sin;
            int k, nsees, ssees;
            static double alpha;

            if (alphap)
            {
                alpha = *(double*)alphap;

                if (!s)
                {
                    return 1;
                }
            }

            if (i == -1)
            {
                return 0;
            }

            si = s->neigh[i].simp;
            scn = s->neigh + cdim - 1;
            sin = s->neigh + i;
            nsees = 0;

            for (k = 0; k < cdim; k++) if (s->neigh[k].vert == infinity && k != i)
                {
                    return 1;
                }

            rs = radsq(s);
            rsi = radsq(si);

            if (rs < alpha &&  rsi < alpha)
            {
                return 1;
            }

            swap_points(scn->vert, sin->vert);
            POWERCRUST_NULLIFY(basis_s, s->neigh[i].basis);
            cdim--;
            get_basis_sede(s);
            reduce(&s->normal, infinity, s, cdim);
            rsfi = radsq(s);

            for (k = 0; k < cdim; k++) if (si->neigh[k].simp == s)
                {
                    break;
                }

            ssees = sees(scn->vert, s);

            if (!ssees)
            {
                nsees = sees(si->neigh[k].vert, s);
            }

            swap_points(scn->vert, sin->vert);
            cdim++;
            POWERCRUST_NULLIFY(basis_s, s->normal);
            POWERCRUST_NULLIFY(basis_s, s->neigh[i].basis);

            if (ssees)
            {
                return alpha < rs;
            }

            if (nsees)
            {
                return alpha < rsi;
            }

            assert(rsfi <= rs + FLT_EPSILON && rsfi <= rsi + FLT_EPSILON);

            return alpha <= rsfi;
        }


        void* PowerCrustHull::conv_facetv(simplex* s, void* dum)
        {
            int i;

            for (i = 0; i < cdim; i++) if (s->neigh[i].vert == infinity)
                {
                    return s;
                }

            return NULL;
        }

        void* PowerCrustHull::mark_points(simplex* s, void* dum)
        {
            int i, snum;
            neighbor* sn;

            for (i = 0, sn = s->neigh; i < cdim; i++, sn++)
            {
                if (sn->vert == infinity)
                {
                    continue;
                }

                snum = pc->site_numm(sn->vert);

                if (s->mark)
                {
                    mo[snum] = 1;
                }
                else
                {
                    mi[snum] = 1;
                }
            }

            return NULL;
        }

        void* PowerCrustHull::visit_outside_ashape(simplex* root, visit_func visit)
        {
            return visit_triang_gen((simplex*)visit_hull(root, &GraspStudio::PowerCrust::PowerCrustHull::conv_facetv), visit, &GraspStudio::PowerCrust::PowerCrustHull::alph_test);
        }

        int PowerCrustHull::check_ashape(simplex* root, double alpha)
        {

            int i;

            for (i = 0; i < MAXPOINTS; i++)
            {
                mi[i] = mo[i] = 0;
            }

            visit_hull(root, &GraspStudio::PowerCrust::PowerCrustHull::zero_marks);

            alph_test(0, 0, &alpha);
            visit_outside_ashape(root, &GraspStudio::PowerCrust::PowerCrustHull::one_marks);

            visit_hull(root, &GraspStudio::PowerCrust::PowerCrustHull::mark_points);

            for (i = 0; i < MAXPOINTS; i++) if (mo[i] && !mi[i])
                {
                    return 0;
                }

            return 1;
        }

        double PowerCrustHull::find_alpha(simplex* root)
        {

            int i;
            float al = 0, ah = 0, am = 0;

            for (i = 0; i < pdim; i++)
            {
                ah += (float)((pc->maxs[i] - pc->mins[i]) * (pc->maxs[i] - pc->mins[i]));
            }

            assert(check_ashape(root, ah));

            for (i = 0; i < 17; i++)
            {
                if (check_ashape(root, am = (al + ah) / 2))
                {
                    ah = am;
                }
                else
                {
                    al = am;
                }

                if ((ah - al) / ah < .5)
                {
                    break;
                }
            }

            return 1.1 * ah;
        }






        void PowerCrustHull::vols(fg* f, Tree* t, basis_s* n, int depth)
        {

            static simplex* s;
            static neighbor* sn;
            int tdim = cdim;
            basis_s* nn = 0;
            int signum;
            point nnv;
            double sqq;


            if (!t)
            {
                return;
            }

            if (!s)
            {
                POWERCRUST_NEWL(simplex, s);
                sn = s->neigh;
            }

            cdim = depth;
            s->normal = n;

            if (depth > 1 && sees(t->key, s))
            {
                signum = -1;
            }
            else
            {
                signum = 1;
            }

            cdim = tdim;

            if (t->fgs->dist == 0)
            {
                sn[depth - 1].vert = t->key;
                POWERCRUST_NULLIFY(basis_s, sn[depth - 1].basis);
                cdim = depth;
                get_basis_sede(s);
                cdim = tdim;
                reduce(&nn, infinity, s, depth);
                nnv = nn->vecs;

                if (t->key == infinity || f->dist == Huge || NEARZERO(nnv[rdim - 1]))
                {
                    t->fgs->dist = Huge;
                }
                else
                    t->fgs->dist = Vec_dot_pdim(nnv, nnv)
                                   / 4 / nnv[rdim - 1] / nnv[rdim - 1];

                if (!t->fgs->facets)
                {
                    t->fgs->vol = 1;
                }
                else
                {
                    vols(t->fgs, t->fgs->facets, nn, depth + 1);
                }
            }

            assert(f->dist != Huge || t->fgs->dist == Huge);

            if (t->fgs->dist == Huge || t->fgs->vol == Huge)
            {
                f->vol = Huge;
            }
            else
            {
                sqq = t->fgs->dist - f->dist;

                if (NEARZERO(sqq))
                {
                    f->vol = 0;
                }
                else f->vol += signum
                                   * sqrt(sqq)
                                   * t->fgs->vol
                                   / (cdim - depth + 1);
            }

            vols(f, t->left, n, depth);
            vols(f, t->right, n, depth);

            return;
        }


        void PowerCrustHull::find_volumes(fg* faces_gr, FILE* F)
        {
            if (!faces_gr)
            {
                return;
            }

            vols(faces_gr, faces_gr->facets, 0, 1);
            print_fg(faces_gr, F);
        }



        void PowerCrustHull::set_ch_root(simplex* s)
        {
            ch_root = s;
            return;
        }
        /* set root to s, for purposes of getting normals etc. */


        simplex* PowerCrustHull::build_convex_hull(gsitef get_s, site_n site_numm, short dim, short vdd)
        {

            /*
              get_s     returns next site each call;
              hull construction stops when NULL returned;
              site_numm returns number of site when given site;
              dim       dimension of point set;
              vdd       if (vdd) then return Delaunay triangulation


            */

#ifdef HULL_DEBUG_TO_FILE
            logHull.logString(" -- build_convex_hull --");
#endif


            simplex* s, *root;

            //   if (!Huge)
            //      Huge = DBL_MAX*DBL_MAX;

            cdim = 0;
            //get_site = get_s;
            //site_num = site_numm;
            pdim = dim;
            vd = vdd;

            exact_bits = (int)(DBL_MANT_DIG * log((float)FLT_RADIX) / log(2.0f));
            b_err_min = (float)(DBL_EPSILON * MAXDIM * (1 << MAXDIM) * MAXDIM * 3.01);
            b_err_min_sq = b_err_min * b_err_min;
            b_err_min = 1.0950316e-011f;
            b_err_min_sq = 1.1990942e-022f;

#ifdef HULL_DEBUG_TO_FILE
            logHull.logInt(exact_bits);
            logHull.logFloat(b_err_min);
            logHull.logFloat(b_err_min_sq);
            logHull.logFloat(b_err_min);
            logHull.logFloat(b_err_min_sq);
#endif


            assert(get_s != NULL);
            assert(site_numm != NULL);

            rdim = vd ? pdim + 1 : pdim;

            if (rdim > MAXDIM)
                panic("dimension bound MAXDIM exceeded; rdim=%d; pdim=%d\n",
                      rdim, pdim);

            /*  fprintf(DFILE, "rdim=%d; pdim=%d\n", rdim, pdim); fflush(DFILE);*/
#ifdef HULL_DEBUG_TO_FILE
            logHull.logInt(rdim);
#endif
            //point_size = site_size = sizeof(Coord)*pdim;
            basis_vec_size = sizeof(Coord) * rdim;
            basis_s_size = sizeof(basis_s) + (2 * rdim - 1) * sizeof(Coord);
            simplex_size = sizeof(simplex) + (rdim - 1) * sizeof(neighbor);
            Tree_size = sizeof(Tree);
            fg_size = sizeof(fg);

            root = NULL;

            if (vd || pc->power_diagram)
            {

                pc->p = infinity;
                POWERCRUST_NEWLRC(basis_s, infinity_basis);
                infinity_basis->vecs[2 * rdim - 1]
                    = infinity_basis->vecs[rdim - 1]
                      = 1;
                infinity_basis->sqa
                    = infinity_basis->sqb
                      = 1;
#ifdef HULL_DEBUG_TO_FILE
                logHull.logInt(2 * rdim - 1, ",", false);
                logHull.logInt(rdim - 1, ",", false);
#endif
            }
            else if (!(pc->p = (*pc.*get_s)()))
            {
#ifdef HULL_DEBUG_TO_FILE
                logHull.logString("Return 0");
#endif
                return 0;
            }

            POWERCRUST_NEWL(simplex, root);

            ch_root = root;

            POWERCRUST_copy_simp(s, root);
            root->peak.vert = pc->p;
            root->peak.simp = s;
            s->peak.simp = root;

            buildhull(root);
            return root;
        }


        void PowerCrustHull::free_hull_storage(void)
        {
            free_basis_s_storage();
            free_simplex_storage();
            free_Tree_storage();
            free_fg_storage();
        }



    }
}
