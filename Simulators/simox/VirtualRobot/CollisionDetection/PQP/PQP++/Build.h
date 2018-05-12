/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk, E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#ifndef PQP_BUILD_H
#define PQP_BUILD_H

#include "PQP.h"

namespace PQP
{

    class Builder
    {
    public:
        int
        build_model(PQP_Model* m);

    private:
        PQP_REAL max(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d);
        PQP_REAL min(PQP_REAL a, PQP_REAL b, PQP_REAL c, PQP_REAL d);
        void get_centroid_triverts(PQP_REAL c[3], Tri* tris, int num_tris);
        void get_covariance_triverts(PQP_REAL M[3][3], Tri* tris, int num_tris);
        int split_tris(Tri* tris, int num_tris, PQP_REAL a[3], PQP_REAL c);
        int build_recurse(PQP_Model* m, int bn, int first_tri, int num_tris);
        void make_parent_relative(PQP_Model* m, int bn, const PQP_REAL parentR[3][3]
#if PQP_BV_TYPE & RSS_TYPE
                                  , const PQP_REAL parentTr[3]
#endif
#if PQP_BV_TYPE & OBB_TYPE
                                  , const PQP_REAL parentTo[3]
#endif
                                 );

        MatVec pqp_math;
    };
}

#endif
