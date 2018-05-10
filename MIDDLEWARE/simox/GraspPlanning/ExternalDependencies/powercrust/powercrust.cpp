
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

#include "powercrust.h"
#include "randStuff.h"
#include "hull.h"

//#define DEBUG_TO_FILE
//#define DEBUG_TO_FILE_FILENAME "c:\\tmp\\powercrust\\1\\powercrust_simox.txt"


#ifdef DEBUG_TO_FILE
#include "CustomLogger.h"
CustomLogger logger(CustomLogger::eFile, DEBUG_TO_FILE_FILENAME);
#endif

using namespace std;

#ifdef WIN32
#pragma warning(push)
#pragma warning(disable: 4996) //4996 for _CRT_SECURE_NO_WARNINGS equivalent
#endif


namespace GraspStudio
{

    namespace PowerCrust
    {

        PowerCrust::PowerCrust()
        {
            init();
        }

        void PowerCrust::init()
        {
            seed = 1; // test
            est_r = 0.6;   /* estimated value of r - user input */
            bad = 0;

            numbadpoles = 0;
            poleid = 0;
            num_vtxs = 0;
            num_faces = 0;
            num_poles = 0;
            num_axedgs = 0;
            num_axfaces = 0;
            num_blocks = 0;
            // we assume 3d points!
            dim = 3;

            vd = 1;
            power_diagram = 0; /* 1 if power diagram */
            s_num = 0; /* site number */
            theta = 0.0; /* input argument - angle defining deep intersection */
            deep = 0.0; /* input argument.. same as theta for labeling unlabled pole */
            defer = 0; /* input argument -D 1 if you don't want to propagate bad poles */
            poleInput = 0; /* are the poles given as input */

            generateOutput = true; // write to file


            for (int i = 0; i < MAXDIM; i++)
            {
                mins[i] = DBL_MAX;
                maxs[i] = -DBL_MAX;
            }

            /* some default values */
            mult_up = 100000;
            est_r = 1;
            DFILE = stderr;

            //MP 2013-08-27
            //bypass file i/o
            resultingPolarBalls.clear();

#ifdef DEBUG_TO_FILE
            logger.logString("Logging PowerCrust within Simox");
            logger.logTimeStamp(true);
#endif
        }

        site PowerCrust::new_site(site p, long j)
        {

            assert(num_blocks + 1 < MAXBLOCKS);

            if (0 == (j % BLOCKSIZE))
            {
                assert(num_blocks < MAXBLOCKS);
                //&(site_blocks[0][5258])
                return (site_blocks[num_blocks++] = (site)malloc(BLOCKSIZE * site_size));
            }
            else
            {
                return p + dim;
            }
        }

        site PowerCrust::buildSite(long j, const Eigen::Vector3f& v)
        {
            if (j != -1)
            {
                p = new_site(p, j);
#ifdef DEBUG_TO_FILE
                logger.logString("buildSite: j=", "", false);
                logger.logLong(j, "", false);
                logger.logString(": ", "", false);
#endif

                for (int i = 0; i < 3; i++)
                {
                    p[i] = floor(mult_up * v(i) + 0.5);
                    mins[i] = (mins[i] < p[i]) ? mins[i] : p[i];
                    maxs[i] = (maxs[i] > p[i]) ? maxs[i] : p[i];
#ifdef DEBUG_TO_FILE
                    logger.logDouble(p[i], ",", false);
                    logger.logDouble(mins[i], ",", false);
                    logger.logDouble(maxs[i], ",", false);
#endif
                }

#ifdef DEBUG_TO_FILE
                logger.logNewLine();
#endif
            }
            else
            {
#ifdef DEBUG_TO_FILE
                logger.logString("buildSite: j==-1");
#endif
            }

            return p;
        }

        site PowerCrust::read_next_site(long j)
        {

            int i = 0, k = 0;
            static char buf[100], *s;

            if (j != -1)
            {
                p = new_site(p, j);
            }

            if (j != 0) while ((s = fgets(buf, sizeof(buf), INFILE)))
                {
                    if (buf[0] == '%')
                    {
                        continue;
                    }

                    for (k = 0; buf[k] && isspace(buf[k]); k++);

                    if (buf[k])
                    {
                        break;
                    }
                }

            if (!s)
            {
                return 0;
            }

            if (j != 0)
            {
                assert(TFILE != NULL);
                fprintf(TFILE, "%s", &(buf[k]));
                fflush(TFILE);
            }

            while (buf[k])
            {
                while (buf[k] && isspace(buf[k]))
                {
                    k++;
                }

                if (buf[k] && j != -1)
                {
                    if (sscanf(buf + k, "%lf", p + i) == EOF)
                    {
                        //fprintf(DFILE, "bad input line: %s\n", buf);
                        THROW_VR_EXCEPTION("bad input line:" << buf);
                    }

                    p[i] = floor(mult_up * p[i] + 0.5);
                    mins[i] = (mins[i] < p[i]) ? mins[i] : p[i];
                    maxs[i] = (maxs[i] > p[i]) ? maxs[i] : p[i];
                }

                if (buf[k])
                {
                    i++;
                }

                while (buf[k] && !isspace(buf[k]))
                {
                    k++;
                }
            }

            return p;
        }



        bool PowerCrust::buildBoundingBox(long j)
        {
            int i, k;
            double center[3], width;
#ifdef DEBUG_TO_FILE
            logger.logString("--- BBox ---", "", true);
#endif
            omaxs[0] = maxs[0];
            omins[0] = mins[0];
            omaxs[1] = maxs[1];
            omins[1] = mins[1];
            omaxs[2] = maxs[2];
            omins[2] = mins[2];

            center[0] = (maxs[0] - mins[0]) / 2;
            center[1] = (maxs[1] - mins[1]) / 2;
            center[2] = (maxs[2] - mins[2]) / 2;

            if ((maxs[0] - mins[0]) > (maxs[1] - mins[1]))
            {
                if ((maxs[2] - mins[2]) > (maxs[0] - mins[0]))
                {
                    width = maxs[2] - mins[2];
                }
                else
                {
                    width = maxs[0] - mins[0];
                }
            }
            else
            {
                if ((maxs[1] - mins[1]) > (maxs[2] - mins[2]))
                {
                    width = maxs[1] - mins[1];
                }
                else
                {
                    width = maxs[2] - mins[2];
                }
            }

            width = width * 4;

            bound[0][0] = center[0] + width;
            bound[1][0] = bound[0][0];
            bound[2][0] = bound[0][0];
            bound[3][0] = bound[0][0];
            bound[0][1] = center[1] + width;
            bound[1][1] = bound[0][1];
            bound[4][1] = bound[0][1];
            bound[5][1] = bound[0][1];
            bound[0][2] = center[2] + width;
            bound[2][2] = bound[0][2];
            bound[4][2] = bound[0][2];
            bound[6][2] = bound[0][2];
            bound[4][0] = center[0] - width;
            bound[5][0] = bound[4][0];
            bound[6][0] = bound[4][0];
            bound[7][0] = bound[4][0];
            bound[2][1] = center[1] - width;
            bound[3][1] = bound[2][1];
            bound[6][1] = bound[2][1];
            bound[7][1] = bound[2][1];
            bound[1][2] = center[2] - width;
            bound[3][2] = bound[1][2];
            bound[5][2] = bound[1][2];
            bound[7][2] = bound[1][2];
#ifdef DEBUG_TO_FILE

            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 3; j++)
                {
                    logger.logDouble(bound[i][j], ",", false);
                }

            logger.logNewLine();
#endif

            for (i = 0; i < 8; i++)
                fprintf(DFILE, "%f %f %f\n",
                        bound[i][0] / mult_up, bound[i][1] / mult_up, bound[i][2] / mult_up);

            for (k = 0; k < 3; k++)
            {
                p[k] = bound[0][k];
            }

            for (int u = 1; u < 8; u++)
            {
                p = new_site(p, j + u);

                for (k = 0; k < 3; k++)
                {
                    p[k] = bound[u][k];
#ifdef DEBUG_TO_FILE
                    logger.log(u);
                    logger.log(k);
                    logger.logDouble(p[k], ",", false);
                    logger.logDouble(bound[u][k], ",", false);
#endif
                }

#ifdef DEBUG_TO_FILE
                logger.logNewLine();
#endif
            }

            num_sites += 8;

            maxs[0] = bound[0][0];
            mins[0] = bound[4][0];
            maxs[1] = bound[0][1];
            mins[1] = bound[2][1];
            maxs[2] = bound[0][2];
            mins[2] = bound[1][2];
#ifdef DEBUG_TO_FILE

            for (int j = 0; j < 3; j++)
            {
                logger.logDouble(mins[j], ",", false);
                logger.logDouble(maxs[j], ",", false);
            }

            logger.logNewLine();
#endif
            return true;
        }


        /* reads a site from storage we're managing outselves */
        site PowerCrust::get_site_offline(long i)
        {
            if (i >= num_sites)
            {
                return NULL;
            }
            else
            {
                return site_blocks[i / BLOCKSIZE] + (i % BLOCKSIZE) * dim;
            }
        }




        void PowerCrust::make_shuffle(void)
        {
#ifdef DEBUG_TO_FILE
            logger.logString("-- make_shuffle --");
#endif
            long i, t, j;
            static long mat_size = 0;
            std::cout << "make_shuffle() 1: mat_size " << mat_size << " num_sites " << num_sites << std::endl; //MP 2013-10-16

            if (mat_size <= num_sites)
            {
                std::cout << "make_shuffle(): allocate array shufmat" << std::endl; //MP 2013-10-16
                mat_size = num_sites + 1;
                shufmat = (long*)malloc(mat_size * sizeof(long));
            }

            std::cout << "make_shuffle() 2: mat_size " << mat_size << " num_sites " << num_sites << std::endl; //MP 2013-10-16

            for (i = 0; i <= num_sites; i++)
            {
                shufmat[i] = i;
            }

            for (i = 0; i < num_sites; i++)
            {
                t = shufmat[i];
                j = i + (long)((num_sites - i) * double_rand());
                shufmat[i] = shufmat[j];
                shufmat[j] = t;
#ifdef DEBUG_TO_FILE
                logger.logLong(shufmat[i], ",", false);
#endif
            }

#ifdef DEBUG_TO_FILE
            logger.logNewLine();
#endif
        }

        void PowerCrust::make_shuffle_repaired_MP(void)
        {
#ifdef DEBUG_TO_FILE
            logger.logString("-- make_shuffle_repaired_MP --");
#endif
            long i, t, j;
            //static long mat_size = 0; //MP removed this! Now using a member instead!
            std::cout << "make_shuffle_repaired_MP() 1: mat_size " << mat_size
                      << " num_sites " << num_sites << std::endl; //MP 2013-10-16

            if (mat_size <= num_sites)
            {
                std::cout << "make_shuffle_repaired_MP(): allocate array shufmat" << std::endl; //MP 2013-10-16
                mat_size = num_sites + 1;
                shufmat = (long*)malloc(mat_size * sizeof(long));
            }

            std::cout << "make_shuffle_repaired_MP() 2: mat_size " << mat_size
                      << " num_sites " << num_sites << std::endl; //MP 2013-10-16

            for (i = 0; i <= num_sites; i++)
            {
                shufmat[i] = i;
            }

            for (i = 0; i < num_sites; i++)
            {
                t = shufmat[i];
                j = i + (long)((num_sites - i) * double_rand());
                shufmat[i] = shufmat[j];
                shufmat[j] = t;
#ifdef DEBUG_TO_FILE
                logger.logLong(shufmat[i], ",", false);
#endif
            }

#ifdef DEBUG_TO_FILE
            logger.logNewLine();
#endif
        }

        //static long (*shuf)(long);
        long PowerCrust::noshuffle(long i)
        {
            return i;
        }

        long PowerCrust::shufflef(long i)
        {
            VR_ASSERT(i >= 0 && i <= num_sites); // i==num_sites -> shufmat[i]==i -> loop break
            return shufmat[i];
        }


        /* returns shuffled, offline sites or reads an unshuffled site, depending on
           how get_site_n and shuf are set up. */
        site PowerCrust::get_next_site(void)
        {
            /*  static long s_num = 0; */
            //return (*get_site_n)((*shuf)(s_num++));
            return get_site_offline(shufflef(s_num++));
        }


        long PowerCrust::site_numm(site p)
        {
            long i, j;

            if ((vd || power_diagram) && p == hull->infinity)
            {
                return -1;
            }

            if (!p)
            {
                return -2;
            }

            for (i = 0; i < num_blocks; i++)
            {
                if ((j = p - site_blocks[i]) >= 0 && j < BLOCKSIZE * dim)
                {
                    return j / dim + BLOCKSIZE * i;
                }
            }

            return -3;
        }


        void PowerCrust::make_output(simplex* root,
                                     void* (PowerCrustHull::*visit_gen)(simplex*, visit_func visit),
                                     visit_func visit,
                                     out_func out_funcp,
                                     FILE* F,
                                     boost::shared_ptr<PowerCrustHull> hull)
        {
            (*hull.*out_funcp)(0, 0, F, -1);
            (*hull.*visit)(0, &out_funcp);
            (*hull.*visit_gen)(root, visit);
            (*hull.*out_funcp)(0, 0, F, 1);
            /*  efclose(F); */
        }


        bool PowerCrust::buildPoles2()
        {
            pr = &GraspStudio::PowerCrust::PowerCrustHull::compute_pole2;
            fprintf(DFILE, "\n\n\ncomputing 2nd poles....\n");
            make_output(root, &GraspStudio::PowerCrust::PowerCrustHull::visit_hull, pr, mof, OUTFILE, hull);


            /* poles with weights. Input to regular triangulation */
            SPFILE = fopen("sp", "w");

            /*  fprintf(POLE,"%s \n","OFF"); */

            /* initialize the sample distance info for the poles */

            pole1_distance = (double*) malloc(num_sites * sizeof(double));
            pole2_distance = (double*) malloc(num_sites * sizeof(double));

            hull->compute_distance(pole1, num_sites - 8, pole1_distance);
            hull->compute_distance(pole2, num_sites - 8, pole2_distance);


            /* intialize list of lists of pointers to opposite poles */
            opplist = (struct plist**) calloc(num_sites * 2, sizeof(struct plist*));

            /* data about poles; adjacencies, labels, radii */
            adjlist = (struct polelabel*) calloc(num_sites * 2, sizeof(struct polelabel));

            /* loop through sites, writing out poles */
            for (int i = 0; i < num_sites - 8; i++)
            {

                /* rescale the sample to real input coordinates */
                for (int k = 0; k < 3; k++)
                {
                    samp[k] = get_site_offline(i)[k] / mult_up;
                }

                /* output poles, both to debugging file and for weighted DT */
                /* remembers sqaured radius */
                if ((pole1[i] != NULL) && (pole1[i]->status != POLE_OUTPUT))
                {
                    /* if second pole is closer than we think it should be... */
                    if ((pole2[i] != NULL) && bad &&
                        hull->close_pole(samp, pole2[i]->vv, lfs_lb[i]))
                    {
                        numbadpoles++;
                    }
                    else
                    {
                        hull->outputPole(POLE, SPFILE, pole1[i], poleid++, samp, &num_poles, pole1_distance[i]);
                    }
                }

                if ((pole2[i] != NULL) && (pole2[i]->status != POLE_OUTPUT))
                {

                    /* if pole is closer than we think it should be... */
                    if (hull->close_pole(samp, pole2[i]->vv, lfs_lb[i]))
                    {
                        /* remember opposite bad for late labeling */
                        if (!bad)
                        {
                            adjlist[pole1[i]->poleindex].bad = BAD_POLE;
                        }

                        numbadpoles++;
                        continue;
                    }

                    /* otherwise... */
                    hull->outputPole(POLE, SPFILE, pole2[i], poleid++, samp, &num_poles, pole2_distance[i]);
                }

                /* keep list of opposite poles for later coloring */
                if ((pole1[i] != NULL) && (pole2[i] != NULL) &&
                    (pole1[i]->status == POLE_OUTPUT) &&
                    (pole2[i]->status == POLE_OUTPUT))
                {

                    pole_angle =
                        hull->computePoleAngle(pole1[i], pole2[i], samp);

                    hull->newOpposite(pole1[i]->poleindex,
                                      pole2[i]->poleindex, pole_angle);
                    hull->newOpposite(pole2[i]->poleindex,
                                      pole1[i]->poleindex, pole_angle);
                }
            }

            hull->efclose(POLE);
            hull->efclose(SPFILE);
            fprintf(DFILE, "bad poles=%d\n", numbadpoles);

            hull->free_hull_storage();
            return true;
        }

        bool PowerCrust::buildPoles1()
        {
            /* Step 2: Find poles */
            pole1 = (struct simplex**) calloc(num_sites, sizeof(struct simplex*));
            pole2 = (struct simplex**) calloc(num_sites, sizeof(struct simplex*));
            lfs_lb = (double*) calloc(num_sites, sizeof(double));

            fprintf(DFILE, "done\n");
            fflush(DFILE);
            /*
                rverts = select_random_points(num_sites);
                fprintf(DFILE, "selecing random points\n");
            */

            mof = &GraspStudio::PowerCrust::PowerCrustHull::vlist_out;//out_funcs[main_out_form];
            pr = &GraspStudio::PowerCrust::PowerCrustHull::facets_print;

            //if  (main_out_form==0) echo_command_line(OUTFILE,argc,argv);

            exactinit(); /* Shewchuk's exact arithmetic initialization */

            pr = &GraspStudio::PowerCrust::PowerCrustHull::compute_vv;
            fprintf(DFILE, "Computing Voronoi vertices and 1st poles....\n");
            make_output(root, &GraspStudio::PowerCrust::PowerCrustHull::visit_hull, pr, mof, OUTFILE, hull);
            return true;
        }

        bool PowerCrust::buildConvexHull()
        {
            // create hull object
            hull.reset(new PowerCrustHull(shared_from_this()));

            /* Step 1: compute DT of input point set */
            root = hull->build_convex_hull(&GraspStudio::PowerCrust::PowerCrust::get_next_site, &GraspStudio::PowerCrust::PowerCrust::site_numm, dim, vd);
            return true;
        }



        bool PowerCrust::buildPowerVV()
        {
            power_diagram = 1;
            vd = 0;
            dim = 4;

            INFILE = fopen("sp", "r");
            fprintf(DFILE, "num_blocks = %d\n", num_blocks);
            /* for (i=0;i<num_blocks;i++) free(site_blocks[i]);*/

            num_blocks = 0;
            s_num = 0;
            scount = 0;
            read_next_site(-1);
            fprintf(DFILE, "dim=%d\n", dim);
            fflush(DFILE);
            /* if (dim > MAXDIM) panic("dimension bound MAXDIM exceeded"); */

            point_size = site_size = sizeof(Coord) * dim;

            /* save points in order read */
            for (num_sites = 0; read_next_site(num_sites); num_sites++);

            fprintf(DFILE, "done; num_sites=%ld\n", num_sites);
            fflush(DFILE);
            hull->efclose(INFILE);

            /* set up the shuffle */
            fprintf(DFILE, "shuffling...");
            init_rand(seed);

            //make_shuffle();   //MP replace this method
            make_shuffle_repaired_MP(); //MP

            shuf = &GraspStudio::PowerCrust::PowerCrust::shufflef;
            get_site_n = &GraspStudio::PowerCrust::PowerCrust::get_site_offline;  /* returns stored points, unshuffled */

            /* Compute weighted DT  */
            root = hull->build_convex_hull(&GraspStudio::PowerCrust::PowerCrust::get_next_site, &GraspStudio::PowerCrust::PowerCrust::site_numm, dim, vd);

            fprintf(DFILE, "scount=%d, s_num=%ld\n", scount, s_num);

            fprintf(DFILE, "done\n");
            fflush(DFILE);

            /* file of faces */
            PNF = fopen("pnf", "w");
            /* file of points */
            PC = fopen("pc", "w");

            /* compute adjacencies and find angles of ball intersections */
            queue = NULL;
            //pr = compute_3d_power_vv;
            make_output(root, &GraspStudio::PowerCrust::PowerCrustHull::visit_hull, &GraspStudio::PowerCrust::PowerCrustHull::compute_3d_power_vv, mof, OUTFILE, hull);

            return true;
        }

        bool PowerCrust::buildMedialAxis()
        {
            /* compute the medial axis */
            //pr=compute_axis;
            fprintf(DFILE, "\n\n computing the medial axis ....\n");
            make_output(root, &GraspStudio::PowerCrust::PowerCrustHull::visit_hull, &GraspStudio::PowerCrust::PowerCrustHull::compute_axis, mof, OUTFILE, hull);

            HEAD = fopen("head", "w");
            fprintf(HEAD, "OFF\n");
            fprintf(HEAD, "%d %d %d\n", num_poles, num_axedgs, 0);
            hull->efclose(HEAD);
            hull->efclose(AXIS);
#ifdef WIN32
            system("type head pole axis > axis.off");
#else
            system("cat head pole axis > axis.off");
#endif

            HEAD = fopen("head", "w");
            fprintf(HEAD, "%d %d \n", num_poles, num_axedgs);
            hull->efclose(HEAD);


#ifdef WIN32
            system("type head tpoleinfo axis > poleinfo");
#else
            system("cat head tpoleinfo axis > poleinfo");
#endif

            HEAD = fopen("head", "w");
            fprintf(HEAD, "OFF\n");
            fprintf(HEAD, "%d %d %d\n", num_poles, num_axfaces, 0);
            hull->efclose(HEAD);
            hull->efclose(AXISFACE);


#ifdef WIN32
            system("type head pole axisface > axisface.off");
#else
            system("cat head pole axisface > axisface.off");
            system("rm -f head pole axis axisface tpoleinfo sp");
#endif


            /* power shape output done */

            hull->efclose(INPOLE);
            hull->efclose(OUTPOLE);
            hull->efclose(INPBALL);
            hull->efclose(TFILE);
            free(adjlist);

            hull->free_hull_storage();
            //hull->efclose(DFILE);

            return true;
        }

        bool PowerCrust::buildCrust()
        {


            /* Enough labeling; let's look at the poles and output a crust!  */
            INPOLE = fopen("inpole", "w");
            OUTPOLE = fopen("outpole", "w");

            /* for visualization of polar balls: */
            INPBALL = fopen("inpball", "w"); /* inner poles with radii */
            POLEINFO = fopen("tpoleinfo", "w");

            //MP 2013 modifications
            FILE* INPBALL_EXT;
            INPBALL_EXT = fopen("inpball_extended", "w"); /* inner polar balls with radii and four surface points (MP)*/

            //MP 2013-08-27 output with file i/o:
            PolarBall temporaryPolarBall;


            double tmp_pt[3];
            struct edgesimp* eindex;

            for (int i = 0; i < num_poles; i++)
            {

                for (int k = 0; k < 3; k++)
                {
                    tmp_pt[k] = get_site_offline(i)[k] / mult_up;
                }

                fprintf(POLEINFO, "%f %f %f %f %d %f \n ", tmp_pt[0], tmp_pt[1], tmp_pt[2],
                        adjlist[i].sqradius, adjlist[i].label, adjlist[i].samp_distance);

                if ((adjlist[i].label != HULL_IN) && (adjlist[i].label != HULL_OUT))
                {
                    fprintf(DFILE, "pole %d label %d\n", i, adjlist[i].label);
                }
                else
                {

                    if (adjlist[i].label == HULL_IN)
                    {
                        fprintf(INPOLE, "%f %f %f\n", tmp_pt[0], tmp_pt[1], tmp_pt[2]);
                        fprintf(INPBALL, "%f %f %f %f \n",
                                tmp_pt[0], tmp_pt[1], tmp_pt[2], sqrt(adjlist[i].sqradius));

                        //MP 2013 modifications: extra information
                        fprintf(INPBALL_EXT, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                                tmp_pt[0], tmp_pt[1], tmp_pt[2], sqrt(adjlist[i].sqradius),
                                adjlist[i].surface_points[0][0], adjlist[i].surface_points[0][1], adjlist[i].surface_points[0][2],
                                adjlist[i].surface_points[1][0], adjlist[i].surface_points[1][1], adjlist[i].surface_points[1][2],
                                adjlist[i].surface_points[2][0], adjlist[i].surface_points[2][1], adjlist[i].surface_points[2][2],
                                adjlist[i].surface_points[3][0], adjlist[i].surface_points[3][1], adjlist[i].surface_points[3][2]);

                        //MP 2013-08-27
                        //put polar balls into a vector for output without files...
                        temporaryPolarBall.poleCenter[0] = tmp_pt[0];
                        temporaryPolarBall.poleCenter[1] = tmp_pt[1];
                        temporaryPolarBall.poleCenter[2] = tmp_pt[2];
                        temporaryPolarBall.poleRadius = sqrt(adjlist[i].sqradius);
                        temporaryPolarBall.surfacePoint1[0] = adjlist[i].surface_points[0][0];
                        temporaryPolarBall.surfacePoint1[1] = adjlist[i].surface_points[0][1];
                        temporaryPolarBall.surfacePoint1[2] = adjlist[i].surface_points[0][2];
                        temporaryPolarBall.surfacePoint2[0] = adjlist[i].surface_points[1][0];
                        temporaryPolarBall.surfacePoint2[1] = adjlist[i].surface_points[1][1];
                        temporaryPolarBall.surfacePoint2[2] = adjlist[i].surface_points[1][2];
                        temporaryPolarBall.surfacePoint3[0] = adjlist[i].surface_points[2][0];
                        temporaryPolarBall.surfacePoint3[1] = adjlist[i].surface_points[2][1];
                        temporaryPolarBall.surfacePoint3[2] = adjlist[i].surface_points[2][2];
                        temporaryPolarBall.surfacePoint4[0] = adjlist[i].surface_points[3][0];
                        temporaryPolarBall.surfacePoint4[1] = adjlist[i].surface_points[3][1];
                        temporaryPolarBall.surfacePoint4[2] = adjlist[i].surface_points[3][2];
                        resultingPolarBalls.push_back(temporaryPolarBall);

                    }
                    else if (adjlist[i].label == HULL_OUT)
                    {
                        fprintf(OUTPOLE, "%f %f %f\n", tmp_pt[0], tmp_pt[1], tmp_pt[2]);
                    }

                    eindex = adjlist[i].eptr;

                    while (eindex != NULL)
                    {
                        if ((i < eindex->pid) &&
                            (hull->antiLabel(adjlist[i].label) == adjlist[eindex->pid].label))
                        {
                            hull->construct_face(eindex->simp, eindex->kth);
                        }

                        eindex = eindex->next;
                    }
                }
            }

            hull->efclose(PC);
            hull->efclose(PNF);
            hull->efclose(POLEINFO);

            /* powercrust output done... */
            HEAD = fopen("head", "w");
            fprintf(HEAD, "OFF\n");
            fprintf(HEAD, "%d %d %d\n", num_vtxs, num_faces, 0);
            hull->efclose(HEAD);

            //MP 2013 modifications
            hull->efclose(INPBALL_EXT);

#ifdef WIN32
            system("type head pc pnf > pc.off");
#else
            system("cat head pc pnf > pc.off");
            system("rm head pc pnf");
#endif
            return true;
        }

        bool PowerCrust::computeMedialAxis()
        {
            buildConvexHull();
            buildPoles1();
            buildPoles2();
            buildPowerVV();
            buildLabels();
            buildCrust();
            buildMedialAxis();
            return true;
        }


        bool PowerCrust::init(vector<Eigen::Vector3f>& vertices)
        {
            init();
            numbadpoles = 0;
            poleid = 0;
            num_vtxs = 0;
            num_faces = 0;
            num_poles = 0;
            num_axedgs = 0;
            num_axfaces = 0;
            // we assume 3d points!
            dim = 3;

            AXIS = fopen("axis", "w");
            AXISFACE = fopen("axisface", "w");
            POLE = fopen("pole", "w");
            TFILE = fopen("tempFile.txt", "w");

            if (!TFILE)
            {
                fprintf(DFILE, "couldn't open file tempFile.txt mode w\n");
                return false;
            }

            if (generateOutput)
            {
                OUTFILE = stdout;
                fprintf(DFILE, "main output to %s\n", "stdout");
            }
            else
            {
                fprintf(DFILE, "no main output\n");
            }

            fprintf(DFILE, "dim=%d\n", dim);
            fflush(DFILE);

            if (dim > MAXDIM)
            {
                GRASPSTUDIO_ERROR << "dimension bound MAXDIM exceeded" << endl;
            }

            point_size = site_size = sizeof(Coord) * dim;



            fprintf(DFILE, "generating sites...");
            int skipCount = 0;
            // to debug this class we need to skip first entry as it is done in the original framework
#define SKIP_FIRST_ENTRY
#ifdef SKIP_FIRST_ENTRY
            skipCount = 1;
#endif

            for (num_sites = 0; num_sites < (long)vertices.size() - skipCount; num_sites++)
            {
                buildSite(num_sites, vertices[num_sites]);
            }

            p = new_site(p, num_sites + 1); // build an extra site (this site is needed later !!! do not remove)
            fprintf(DFILE, "done; num_sites=%ld\n", num_sites);
            fflush(DFILE);

            if (!buildBoundingBox(num_sites))
            {
                GRASPSTUDIO_ERROR << "Could not build bbox" << endl;
                return false;
            }

            fprintf(DFILE, "shuffling...");
            init_rand(seed);

            //make_shuffle();   //MP replace this method
            mat_size = 0;   //MP
            make_shuffle_repaired_MP(); //MP

            shuf = &GraspStudio::PowerCrust::PowerCrust::shufflef;
            get_site_n = &GraspStudio::PowerCrust::PowerCrust::get_site_offline;


            return true;
        }

        bool PowerCrust::buildLabels()
        {


            /* Begin by labeling everything outside a big bounding box as outside */

            /* labeling */
            if (!poleInput)  /* if we dont have the labels */
            {
                fprintf(DFILE, "num_poles=%d\n", num_poles);
                hull->init_heap(num_poles);

                for (int i = 0; i < num_poles; i++)
                {
                    if ((get_site_offline(i)[0] > (2 * omaxs[0] - omins[0])) ||
                        (get_site_offline(i)[0] < (2 * omins[0] - omaxs[0])) ||
                        (get_site_offline(i)[1] > (2 * omaxs[1] - omins[1])) ||
                        (get_site_offline(i)[1] < (2 * omins[1] - omaxs[1])) ||
                        (get_site_offline(i)[2] > (2 * omaxs[2] - omins[2])) ||
                        (get_site_offline(i)[2] < (2 * omins[2] - omaxs[2])))
                    {
                        adjlist[i].hid = hull->insert_heap(i, 1.0);
                        adjlist[i].out = 1.0;
                        adjlist[i].label = HULL_OUT;
                    }
                }

                while (hull->heap_size != 0)
                {
                    hull->propagate();
                }

                hull->label_unlabeled(num_poles);

            }

            return true;
        }


        void PowerCrust::setVerbose(bool enable)
        {
            if (enable)
            {
                DFILE = stdout;
            }
            else
            {
                DFILE = NULL;
            }

            if (hull)
            {
                hull->DFILE = DFILE;
            }
        }

    }
}
#ifdef WIN32
#pragma warning(pop)
#endif

