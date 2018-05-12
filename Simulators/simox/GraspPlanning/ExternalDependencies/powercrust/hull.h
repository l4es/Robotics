/* hull.h */
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

/*
 * This file is a significant modification of Ken Clarkson's file hull.h
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

#ifndef _POWERCRUST_HDR
#define _POWERCRUST_HDR

#include <cmath>
#include <cfloat>

#include "GraspPlanning/GraspStudio.h"

#include "points.h"
#include "pointsites.h"

namespace GraspStudio
{

    namespace PowerCrust
    {
        class PowerCrustHull;
        class PowerCrust;

        //MP 2013-08-27
        //struct to collect polar balls for easy output without file i/o.
        /*typedef struct PolarBall {
            double poleCenter[3];
            double poleRadius;
            double surfacePoint1[3];
            double surfacePoint2[3];
            double surfacePoint3[3];
            double surfacePoint4[3];
        } PolarBall;*/

        struct PolarBall
        {
            double poleCenter[3];
            double poleRadius;
            double surfacePoint1[3];
            double surfacePoint2[3];
            double surfacePoint3[3];
            double surfacePoint4[3];
        };


        //////////////////////////////////////////////////////////////////////////
        // from ndefs.h
        //////////////////////////////////////////////////////////////////////////
        typedef struct basis_s
        {
            struct basis_s* next; /* free list */
            int ref_count;  /* storage management */
            int lscale;    /* the log base 2 of total scaling of vector */
            Coord sqa, sqb; /* sums of squared norms of a part and b part */
            Coord vecs[1]; /* the actual vectors, extended by malloc'ing bigger */
        } basis_s;
        //POWERCRUST_STORAGE_GLOBALS(basis_s)


        typedef struct neighbor
        {
            site vert; /* vertex of simplex */
            /*        short edgestatus[3];  FIRST_EDGE if not visited
                      NOT_POW if not dual to powercrust faces
                      POW if dual to powercrust faces */
            struct simplex* simp; /* neighbor sharing all vertices but vert */
            basis_s* basis; /* derived vectors */
        } neighbor;



        typedef struct simplex
        {
            struct simplex* next;   /* used in free list */
            short mark;
            site vv; /* Voronoi vertex of simplex ; sunghee */
            double sqradius; /* squared radius of Voronoi ball */
            /*        site av; */ /* averaged pole */
            /*        double cond; */
            /*    float Sb; */
            short status;/* sunghee : 0(CNV) if on conv hull so vv contains normal vector;
                        1(VV) if vv points to circumcenter of simplex;
                        -1(SLV) if cond=0 so vv points to hull
                        2(AV) if av contains averaged pole */
            long poleindex; /* for 1st DT, if status==POLE_OUTPUT, contains poleindex; for 2nd, contains vertex index for powercrust output for OFF file format */
            short edgestatus[6]; /* edge status :(01)(02)(03)(12)(13)(23)
                                FIRST_EDGE if not visited
                                VISITED
                                NOT_POW if not dual to powercrust faces
                                POW if dual to powercrust faces */
            /*  short tristatus[4];   triangle status :
                FIRST if not visited
                NO   if not a triangle
                DEG  if degenerate triangle
                SURF if surface triangle
                NORM if fails normal test
                VOR  if falis voronoi edge test
                VOR_NORM if fails both test */
            /* NOTE!!! neighbors has to be the LAST field in the simplex stucture,
               since it's length gets altered by some tricky Clarkson-move.
               Also peak has to be the one before it.
               Don't try to move these babies!! */
            long visit;     /* number of last site visiting this simplex */
            basis_s* normal;    /* normal vector pointing inward */
            neighbor peak;      /* if null, remaining vertices give facet */
            neighbor neigh[1];  /* neighbors of simplex */
        } simplex;
        //POWERCRUST_STORAGE_GLOBALS(simplex)

        typedef void* (PowerCrustHull::*visit_func)(simplex*, void*);
        typedef int (PowerCrustHull::*test_func)(simplex*, int, void*);
        typedef void (PowerCrustHull::*out_func)(point*, int, FILE*, int);
        typedef void (PowerCrustHull::*print_neighbor_f)(FILE*, neighbor*);
        typedef site(PowerCrust::*gsitef)(void);
        //extern gsitef *get_site;
        typedef long(PowerCrust::*site_n)(site);
        //extern site_n *site_num;
        //////////////////////////////////////////////////////////////////////////


        /*
        #define DEBS(qq)  {if (DEBUG>qq) {
        #define EDEBS }}
        #define DEBOUT DFILE
        #define DEB(ll,mes)  DEBS(ll) fprintf(DEBOUT,#mes "\n");fflush(DEBOUT); EDEBS
        #define DEBEXP(ll,exp) DEBS(ll) fprintf(DEBOUT,#exp "=%G\n", (double) exp); fflush(DEBOUT); EDEBS
        #define DEBTR(ll) DEBS(ll) fprintf(DEBOUT, __FILE__ " line %d \n" ,__LINE__);fflush(DEBOUT); EDEBS
        #define warning(lev, x)                     \
            {static int messcount;                  \
                if (++messcount<=10) {DEB(lev,x) DEBTR(lev)}    \
                if (messcount==10) DEB(lev, consider yourself warned) \
            }                           \
        */
        /*
        #define SBCHECK(s)                                \
                                                            {double Sb_check=0;                             \
                                                            int i;                                      \
                                                            for (i=1;i<cdim;i++) if (s->neigh[i].basis)             \
                                                            Sb_check+=s->neigh[i].basis->sqb;       \
                                                            if ((float)(Sb_check - s->Sb) !=0.0)                            \
                                                            {DEBTR DEB(bad Sb); DEBEXP(s->Sb) DEBEXP(Sb_check);print_simplex(s); exit(1);}}\
        */





        /* Ravi:  for the thinning stuff */

        /* represent a node in the graph */

        typedef  struct  spole   /* simple version to rep neighbors */
        {
            long index;
            struct spole* next;
        } snode;

        typedef   struct vpole
        {
            long index; /* index of the node */
            long pindex; /* index in the actual list of poles */
            double px;
            double py;
            double pz;
            double pr;  /* the radius of the ball centered here */
            double perpma; /* perpendicular distance from sample to medial axis */
            double pw;
            snode*  adj;
            int status;  /* for thinning */
            int label;  /* might be useful for computing the crust again */
            long substitute; /* if removed points to the substitute node */
            double estlfs; /* the estimated lfs of each ball */
        } vnode ;

        /* edges in the powerface */

        typedef struct enode
        {
            long sindex;
            long dindex;
        }   edge;

        typedef struct fnode
        {
            long index1;
            long index2;
            long index3;
        } face;

        const int power_v1[6] = {0, 0, 0, 1, 1, 2};
        const int power_v2[6] = {1, 2, 3, 2, 3, 3};
        const int power_v3[6] = {2, 3, 1, 3, 0, 0};
        const int power_v4[6] = {3, 1, 2, 0, 2, 1};


        /* end defn for medial axis thinning */


        /* structure for list of opposite poles, opplist. */
        typedef struct plist
        {
            long pid;
            double angle;
            struct plist* next;
        } plist;

        /* regular triangulation edge, between pole pid to center of simp? */
        typedef struct edgesimp
        {
            short kth;
            double angle;   /* angle between balls */
            struct simplex* simp;
            long pid;
            struct edgesimp* next;
        } edgesimp;

        /* additional info about poles: label for pole, pointer to list of regular
           triangulation edges, squared radius of  polar ball. adjlist is an
           array of polelabels. */
        typedef struct polelabel
        {
            struct edgesimp* eptr;
            short bad;
            short label;
            double in; /* 12/7/99 Sunghee for priority queue */
            double out; /* 12/7/99 Sunghee for priority queue */
            int hid; /* 0 if not in the heap, otherwise heap index 1..heap_size*/
            double sqradius;
            double oppradius; /* minimum squared radius of this or any opposite ball */
            double samp_distance;
            int grafindex; /* index in thinning graph data structure */
            double surface_points[4][3]; //MP 2013 modifications: surface points on this polar ball!
        } polelabel;

        typedef struct queuenode
        {
            long pid;
            struct queuenode* next;
        } queuenode;

        typedef struct temp
        {
            struct simplex* simp;
            int vertptr[3];
            int novert;
            /* 0,1,2,3 : 3 vertices but ts->neigh[ti].vert are vertices of triangle */
        } temp;

        typedef struct tarr
        {
            int tempptr[50];
            int num_tempptr;
            long vert;
        } tarr;

        /*
        typedef struct tlist {
          int tempptr;
          struct tlist *next;
        } tlist;
        */

        typedef struct fg_node fg;
        typedef struct tree_node Tree;
        struct tree_node
        {
            Tree* left, *right;
            site key;
            int size;   /* maintained to be the number of nodes rooted here */
            fg* fgs;
            Tree* next; /* freelist */
        };
        //POWERCRUST_STORAGE_GLOBALS(Tree)


        typedef struct fg_node
        {
            Tree* facets;
            double dist, vol;   /* of Voronoi face dual to this */
            fg* next;       /* freelist */
            short mark;
            int ref_count;
        } fg_node;
        //POWERCRUST_STORAGE_GLOBALS(fg)


        const double Huge = DBL_MAX;
        typedef short zerovolf(simplex*);

        /* from fg.c, for face graphs */
        /*
        fg *build_fg(simplex*);

        void print_fg(fg*, FILE *);

        void print_fg_alt(fg*, FILE *, int);
        */
        /* from predicates.c, math.c */
        void normalize(double*);
        double sqdist(double*, double*);
        void dir_and_dist(double*, double*, double*, double*);
        double dotabac(double*, double*, double*);
        double maxsqdist(double*, double*, double*, double*);
        double dotabc(double*, double*, double*);
        void crossabc(double*, double*, double*, double*);
        void tetcircumcenter(double*, double*, double*, double*, double*, double*);
        void tricircumcenter3d(double*, double*, double*, double*, double*);
        void exactinit();
        double orient3d(double*, double*, double*, double*);
        double orient2d(double*, double*, double*);
        void triorthocenter(double a[], double b[], double c[],
                            double orthocenter[], double* cnum);
        void tetorthocenter(double a[], double b[], double c[], double d[], double orthocenter[], double* cnum);
        /* heap.c */
        typedef struct heap_array
        {
            int pid;
            double pri;
        } heap_array;

        /*void init_heap(int);
        void heapify(int);
        int extract_max();
        int insert_heap(int , double);
        void update(int , double);
        */


        /*power.c */
        //int correct_orientation(double*,double*,double*,double*,double*);
        const int MAXPOINTS = 10000;


        class PowerCrustHull
        {
            friend class PowerCrust;
        public:
            PowerCrustHull(boost::shared_ptr<PowerCrust> pc);

        protected:
            Coord infinity[10];//={57.2,0,0,0,0}; /* point at infinity for vd; value not used */
            int pdim;
            //char tmpfilenam[L_tmpnam]; //MP
            char tmpfilenam[10000]; //MP


            FILE* efopen(char*, char*);
            void  efclose(FILE* file);
            FILE* DFILE;


            int
            rdim,   /* region dimension: (max) number of sites specifying region */
            cdim;   /* number of sites currently specifying region */


            /* Ravi thin axis */

            void thinaxis();
            void printaxis();
            void initialize();

            /* from driver, e.g., hullmain.c */

            //typedef site gsitef(void);
            //extern gsitef *get_site;
            //typedef long site_n(site);
            //extern site_n *site_num;
            //site get_site_offline(long); /* sunghee */

            double bound[8][3];

            int power_numvtxs, power_numfaces;
            /* label.c */
            void opp_update(int);
            void sym_update(int);
            void update_pri(int, int);
            int propagate();
            void label_unlabeled(int);


            void construct_face(simplex*, short);

            /* from segt.c or ch.c */

            simplex* build_convex_hull(gsitef, site_n, short, short);

            void free_hull_storage(void);

            int sees(site, simplex*);

            void get_normal(simplex* s);

            int out_of_flat(simplex*, site);

            void set_ch_root(simplex*);

            void print_site(site, FILE*);

            void print_normal(simplex*);

            void* check_marks(simplex*, void*);

            double cosangle_sq(basis_s* v, basis_s* w);

            double find_alpha(simplex*);
            //test_func alph_test;
            void* visit_outside_ashape(simplex*, visit_func);

            void get_basis_sede(simplex*);

            void compute_distance(simplex**, int, double*);

            /* for debugging */
            int check_perps(simplex*);

            void find_volumes(fg*, FILE*);

            short mi[MAXPOINTS], mo[MAXPOINTS];


            /* from hull.c */
            void* visit_triang_gen(simplex*, visit_func, test_func);
            void* visit_triang(simplex*, visit_func);
            void* visit_hull(simplex*, visit_func);

            neighbor* op_simp(simplex* a, simplex* b);

            neighbor* op_vert(simplex* a, site b);

            simplex* new_simp(void);

            void buildhull(simplex*);

            void vlist_out(point*, int, FILE*, int);
            void ps_out(point*, int, FILE*, int);
            void cpr_out(point*, int, FILE*, int);
            void mp_out(point*, int, FILE*, int);
            void off_out(point*, int, FILE*, int);
            void vv_out(point*, int, FILE*, int);
            /* sunghee : added vlist_out */
            /* functions for different formats */

            int crust_loopStart;
            int crust_count;
            int crust_lastCount;
            int crust_FALSE_i;
            int crust_TRUE_i;
            /* added compute axis RAVI */
            void* facets_print(simplex*, void*);
            void* afacets_print(simplex*, void*);
            void* ridges_print(simplex*, void*);
            void* compute_vv(simplex*, void*);
            void* compute_pole1(simplex*, void*);
            void* compute_pole2(simplex*, void*);
            void* test_surface(simplex*, void*);
            void* compute_2d_power_vv(simplex*, void*);
            void* compute_3d_power_vv(simplex*, void*);
            void* compute_3d_power_edges(simplex*, void*);
            void* compute_axis(simplex*, void*);
            /* to print facets, alpha facets, ridges */
            /* Sunghee added compute_cc, compute_pole1, compute_pole2, test_surface */

            void test_temp();

            int correct_orientation(double* p1, double* p2, double* p3, double* inp, double* outp);

            /* Nina's functions in crust.c */
            short is_bound(simplex*);
            int close_pole(double*, double*, double);
            int antiLabel(int);
            int cantLabelAnything(int);
            void labelPole(int, int);
            void newOpposite(int, int, double);
            double computePoleAngle(simplex*, simplex*, double*);
            void outputPole(FILE*, FILE*, simplex*, int, double*, int*, double);

            void print_edge_dat(fg*, FILE*);

            int alph_test(simplex* s, int i, void* alphap);
            /* from pointops.c */

            void print_point(FILE*, int, point);
            void print_point_int(FILE*, int, point);
            Coord maxdist(int, point p1, point p2);
            int reduce(basis_s** v, point p, simplex* s, int k);
            int reduce_inner(basis_s* v, simplex* s, int k);
            double lower_terms_point(point vp);
            double lower_terms(basis_s* v);
            double sc(basis_s* v, simplex* s, int k, int j);
            Coord Vec_dot(point x, point y);
            Coord Vec_dot_pdim(point x, point y);
            Coord Norm2(point x);
            void Ax_plus_y(Coord a, point x, point y);
            void Ax_plus_y_test(Coord a, point x, point y);
            void Vec_scale(int n, Coord a, Coord* x);
            void Vec_scale_test(int n, Coord a, Coord* x);
            void* conv_facetv(simplex* s, void* dum);
            void* mark_points(simplex* s, void* dum);
            int check_ashape(simplex* root, double alpha);
            void vols(fg* f, Tree* t, basis_s* n, int depth);

            void connect(simplex* s);
            simplex* extend_simplices(simplex* s);
            point get_another_site(void);
            simplex* make_facets(simplex* seen);
            simplex* search(simplex* root);
            void get_normal_sede(simplex* s);
            double radsq(simplex* s);
            void* zero_marks(simplex* s, void* dum);
            void* one_marks(simplex* s, void* dum);
            void* show_marks(simplex* s, void* dum);

            // io.cpp

            void panic(char* fmt, ...);

            //print_neighbor_f print_neighbor_full;
            //print_neighbor_f print_neighbor_snum;
            void print_neighbor_snum(FILE* F, neighbor* n);
            void print_neighbor_full(FILE* F, neighbor* n);
            void check_triang(simplex*);


            void check_new_triangs(simplex*);

            void print_extra_facets(void);

            void* print_facet(FILE*, simplex*, print_neighbor_f);

            void print_basis(FILE*, basis_s*);

            void* print_simplex_f(simplex*, FILE*, print_neighbor_f);

            void* print_simplex(simplex*, void*);

            void print_triang(simplex*, FILE*, print_neighbor_f);


            //double SQ(double a);

            /* tables for runtime stats */
            boost::shared_ptr<PowerCrust> pc;
            int A[100], B[100], C[100], D[100];
            int tot , totinf, bigt;
            //gsitef *get_site;
            //site_n *site_num;
            //void print_hist_fg(simplex *, fg*, FILE *);
            //int power_v1[6], power_v2[6], power_v3[6], power_v4[6];

            /*  void arena_check(void); */  /* from hobby's debugging malloc  */

            Tree* splay(site i, Tree* t) /* Splay using the key i (which may or may not be in the tree.) */ /* The starting root is t, and the tree used is defined by rat */ /* size fields are maintained */;
            Tree* insert(site i, Tree* t);
            Tree* deleteT(site i, Tree* t);
            Tree* find_rank(int r, Tree* t);
            void printtree_flat_inner(Tree* t);
            void printtree_flat(Tree* t);
            void printtree(Tree* t, int d);
            fg* find_fg(simplex* s, int q);
            fg* build_fg(simplex* root);
            void visit_fg_i(void (PowerCrustHull::*v_fg)(Tree*, int, int), Tree* t, int depth, int vn, int boundary);
            void visit_fg(fg* faces_gr, void (PowerCrustHull::*v_fg)(Tree*, int, int));
            int visit_fg_i_far(void (PowerCrustHull::*v_fg)(Tree*, int), Tree* t, int depth, int vn);
            void visit_fg_far(fg* faces_gr, void (PowerCrustHull::*v_fg)(Tree*, int));
            void p_fg(Tree* t, int depth, int bad);
            void p_fg_x(Tree* t, int depth, int bad);
            void print_fg_alt(fg* faces_gr, FILE* F, int fd);
            void print_fg(fg* faces_gr, FILE* F);
            void h_fg(Tree* t, int depth, int bad);
            void h_fg_far(Tree* t, int depth);
            void print_hist_fg(simplex* root, fg* faces_gr, FILE* F);
            void* add_to_fg(simplex* s, void* dum);
            int truet(simplex* s, int i, void* dum);
            int hullt(simplex* s, int i, void* dummy);
            void* facet_test(simplex* s, void* dummy);
            void* check_simplex(simplex* s, void* dum);
            int p_neight(simplex* s, int i, void* dum);
            FILE* epopen(char* com, char* mode);
            //void print_neighbor_snum(FILE* F, neighbor *n);
            void print_simplex_num(FILE* F, simplex* s);
            void* p_peak_test(simplex* s);



            struct heap_array* heap_A;
            int heap_length;
            int heap_size;
            void init_heap(int num, FILE* DFILE = NULL);
            void heapify(int hi);
            int extract_max();
            int insert_heap(int pi, double pr);
            void update(int hi, double pr);

            /* for priority queue */
            int LEFT(int i);
            int RIGHT(int i);
            int PARENT(int i);

            double SQ(double a);


        };


        typedef boost::shared_ptr<PowerCrustHull> PowerCrustHullPtr;

    } // namespace PowerCrust

} // namespace GraspStudio

#endif
