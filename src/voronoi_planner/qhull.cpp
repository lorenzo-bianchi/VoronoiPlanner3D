/**
 * VoronoiPlanner - _qhull.pyx implementation.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * January 13, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <voronoi_planner/voronoi_planner.hpp>

namespace VoronoiPlanner
{
// Qhull class implementation
/*  */
Qhull::Qhull(std::string flags, std::vector<Point> points)
{
  this->ndim = points[0].size();                 // dimension of points
  this->numpoints = points.size();               // number of points

  this->point_arrays.push_back(points);

  char* options_c = new char [flags.length()+1];
  std::strcpy(options_c, flags.c_str());

  coordT* coord = nullptr;

  this->qh = (qhT*) malloc(sizeof(qhT));
  qh_zero(this->qh, stderr);

  int exitcode = qh_new_qhull_scipy(this->qh,
                                    this->ndim,
                                    this->numpoints,
                                    (realT*) points.data(),
                                    false,
                                    options_c,
                                    nullptr,
                                    stderr,
                                    coord
                                    );

  if (exitcode != 0) // if error
  {
    this->close();
    throw std::runtime_error("Qhull error");
  }
}

/*  */
Qhull::~Qhull()
{
  int curlong, totlong;

  if (this->qh != nullptr)
  {
    qh_freeqhull(this->qh, qh_ALL);
    qh_memfreeshort(this->qh, &curlong, &totlong);
    free(this->qh);
    this->qh = nullptr;

    if (curlong != 0 or totlong != 0)
      std::cout << "Qhull internal warning (main)" << std::endl;
  }
}

/*  */
void Qhull::check_active()
{
  if (this->qh == nullptr) throw std::runtime_error("Qhull instance is closed");
}

/*  */
void Qhull::close()
{
  int curlong, totlong;

  if (this->qh != nullptr)
  {
    qh_freeqhull(this->qh, qh_ALL);
    qh_memfreeshort(this->qh, &curlong, &totlong);
    free(this->qh);
    this->qh = nullptr;

    if (curlong != 0 or totlong != 0)
      throw std::runtime_error("Qhull internal warning (main)");
  }
}

/*  */
std::vector<Point> Qhull::get_points()
{
  return this->point_arrays[0];
}

/*  */
void Qhull::get_voronoi_diagram(VertexChain& vor_vertices,
                                std::vector<Eigen::Vector2i>& vor_ridge_points,
                                RidgeVertices& vor_ridge_vertices,
                                Chains& vor_regions,
                                std::vector<int>& vor_point_region)
{
  // Return the voronoi diagram currently in Qhull.

  // Returns
  // -------
  // voronoi_vertices : array of double, shape (nvoronoi_vertices, ndim)
  //     Coordinates of the Voronoi vertices

  // ridge_points : array of double, shape (nridges, 2)
  //     Voronoi ridges, as indices to the points array.

  // ridge_vertices : list of lists, shape (nridges, *)
  //     Voronoi vertices for each Voronoi ridge, as indices to
  //     the Voronoi vertices array.
  //     Infinity is indicated by index ``-1``.

  // regions : list of lists, shape (nregion, *)
  //     Voronoi vertices of all regions.

  // point_region : array of int, shape (npoint,)
  //     Index of the Voronoi region for each input point.

  int i, j;
  vertexT* vertex;
  facetT* neighbor;
  facetT* facet;

  VertexChain voronoi_vertices;
  int nvoronoi_vertices;
  pointT* point;
  pointT* center;
  double dist;
  int inf_seen;

  Chains regions;
  Chain cur_region;

  this->check_active();

  // Grab Voronoi ridges
  this->nridges = 0;
  this->ridge_points = RidgeVertices(10);
  this->ridge_vertices = {};

  // io_r.c
  qh_eachvoronoi_all(this->qh, (FILE*) this, &visit_voronoi, this->qh[0].UPPERdelaunay, qh_RIDGEall, 1);

  {
    auto startIt = this->ridge_points.begin();
    auto endIt = this->ridge_points.begin() + this->nridges;
    std::vector<Eigen::Vector2i> tmp(startIt, endIt);
    this->ridge_points = tmp;
  }

  // Now qh_eachvoronoi_all has initialized facets' visitids to correspond to Voronoi vertex indices

  // Grab Voronoi regions
  std::vector<int> point_region(this->numpoints, -1);
  vertex = this->qh[0].vertex_list;

  while (vertex && vertex->next)
  {
    qh_order_vertexneighbors_nd(this->qh, this->ndim+1, vertex);

    i = qh_pointid(this->qh, vertex->point);
    if (i < this->numpoints)
      // Qz results to one extra point
      point_region[i] = regions.size();

    inf_seen = 0;
    cur_region.clear();
    for (int k = 0; k < qh_setsize(this->qh, vertex->neighbors); k++)
    {
      neighbor = (facetT*) vertex->neighbors->e[k].p;
      i = neighbor->visitid - 1;
      if (i == -1)
      {
        if (!inf_seen)
          inf_seen = 1;
        else
          continue;
      }
      cur_region.push_back(i);
    }

    if (cur_region.size() == 1 && cur_region[0] == -1)
        cur_region.clear();
    regions.push_back(cur_region);

    vertex = vertex->next;
  }

  // Grab Voronoi vertices and point-to-region map
  nvoronoi_vertices = 0;

  facet = this->qh[0].facet_list;

  while (facet && facet->next)
  {
    if (facet->visitid > 0)
    {
      // finite Voronoi vertex
      center = qh_facetcenter(this->qh, facet->vertices);

      nvoronoi_vertices = std::max((int) facet->visitid, nvoronoi_vertices);
      if (nvoronoi_vertices >= (int) voronoi_vertices.size())
      {
        auto tmp = voronoi_vertices;
        voronoi_vertices.clear();
        // Array is safe to resize
        tmp.resize(2*nvoronoi_vertices + 1);
        voronoi_vertices = tmp;
      }

      for (int k = 0; k < this->ndim; k++)
        voronoi_vertices[facet->visitid-1][k] = center[k];

      qh_memfree(this->qh, center, this->qh[0].center_size);

      if (facet->coplanarset)
      {
        for (int k = 0; k < qh_setsize(this->qh, facet->coplanarset); k++)
        {
          point = (pointT*) facet->coplanarset->e[k].p;
          vertex = qh_nearvertex(this->qh, facet, point, &dist);
          i = qh_pointid(this->qh, point);
          j = qh_pointid(this->qh, vertex->point);
          if (i < this->numpoints)
            point_region[i] = point_region[j];
        }
      }
    }
    facet = facet->next;
  }

  {
    auto startIt = voronoi_vertices.begin();
    auto endIt = voronoi_vertices.begin() + nvoronoi_vertices;
    voronoi_vertices.assign(startIt, endIt);
  }

  vor_vertices = voronoi_vertices;
  vor_ridge_points = this->ridge_points;
  vor_ridge_vertices = this->ridge_vertices;
  vor_regions = regions;
  vor_point_region = point_region;
}

// Voronoi class implementation
/*  */
Voronoi::Voronoi(std::vector<Point> points)
{
  std::string flags = "qhull v Qbb Qc Qz";
  Qhull qhull = Qhull(flags, points);

  // _QhullUser.__init__(self, qhull, incremental=incremental)
  this->vertices = {};
  this->ridge_points = {};
  this->ridge_vertices = {};
  this->regions = {};
  this->point_region = {};
  qhull.get_voronoi_diagram(this->vertices,
                            this->ridge_points,
                            this->ridge_vertices,
                            this->regions,
                            this->point_region);

  this->points = qhull.get_points();
  this->ndim = this->points[0].size();
  this->npoints = this->points.size();
  auto minIt = std::min_element(this->points.begin(), this->points.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs[0] < rhs[0];
        });
  this->min_bound = *minIt;
  auto maxIt = std::max_element(this->points.begin(), this->points.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs[0] > rhs[0];
        });
  this->max_bound = *maxIt;
  // end _QhullUser.__init__
}

// Functions implementation
/*  */
void visit_voronoi(qhT* _qh, FILE* ptr, vertexT* vertex, vertexT* vertexA, setT* centers, boolT unbounded)
{
  UNUSED(unbounded);

  Qhull* qh = (Qhull*) ptr;
  int point1, point2, ix;

  if (qh->get_nridges() >= (int) qh->ridge_points.size())
  {
    try
    {
      // The array is guaranteed to be safe to resize
      qh->ridge_points.resize(2*qh->get_nridges() + 1);
    }
    catch(std::exception& e)
    {
      return;
    }
  }

  // Record which points the ridge is between
  point1 = qh_pointid(_qh, vertex->point);
  point2 = qh_pointid(_qh, vertexA->point);

  auto& rpoints = qh->ridge_points;

  int* p = (int*) rpoints.data();
  p[2*qh->get_nridges() + 0] = point1;
  p[2*qh->get_nridges() + 1] = point2;

  // Record which voronoi vertices constitute the ridge
  Eigen::Vector2i cur_vertices;
  for (int i = 0; i < qh_setsize(_qh, centers); i++)
  {
    ix = ((facetT*) centers->e[i].p)->visitid - 1;
    cur_vertices[i] = ix;
  }
  qh->ridge_vertices.push_back(cur_vertices);
  qh->set_nridges(qh->get_nridges() + 1);
}

/*  */
void qh_order_vertexneighbors_nd(qhT* qh, int nd, vertexT* vertex)
{
  if (nd == 3)
    qh_order_vertexneighbors(qh, vertex);
  else if (nd >= 4)
    qsort((facetT**) &vertex->neighbors->e[0].p, qh_setsize(qh, vertex->neighbors), sizeof(facetT*), qh_compare_facetvisit);
}

/*  */
int qh_new_qhull_scipy(qhT* qh, int dim, int numpoints, coordT* points, boolT ismalloc,
                       char* qhull_cmd, FILE* outfile, FILE* errfile, coordT *feaspoint)
{
  int exitcode, hulldim;
  boolT new_ismalloc;
  coordT *new_points;

  if (!errfile) errfile = stderr;

  if (!qh->qhmem.ferr) qh_meminit(qh, errfile);
  else qh_memcheck(qh);

  if (strncmp(qhull_cmd, "qhull ", (size_t)6) && strcmp(qhull_cmd, "qhull") != 0)
  {
    qh_fprintf(qh, errfile, 6186, "qhull error (qh_new_qhull): start qhull_cmd argument with \"qhull \" or set to \"qhull\"\n");
    return qh_ERRinput;
  }

  qh_initqhull_start(qh, nullptr, outfile, errfile);
  if(numpoints == 0 && points == nullptr)
  {
    trace1((qh, qh->ferr, 1047, "qh_new_qhull: initialize Qhull\n"));
    return 0;
  }

  trace1((qh, qh->ferr, 1044, "qh_new_qhull: build new Qhull for %d %d-d points with %s\n", numpoints, dim, qhull_cmd));
  exitcode = setjmp(qh->errexit);
  if (!exitcode)
  {
    qh->NOerrexit = False;
    qh_initflags(qh, qhull_cmd);
    if (qh->DELAUNAY) qh->PROJECTdelaunay= True;
    if (qh->HALFspace)
    {
      /* points is an array of halfspaces, the last coordinate of each halfspace is its offset */
      hulldim = dim - 1;
      if (feaspoint)
      {
        coordT* coords;
        coordT* value;
        int i;
        if (!(qh->feasible_point= (pointT*)qh_malloc(hulldim * sizeof(coordT))))
        {
          qh_fprintf(qh, qh->ferr, 6079, "qhull error: insufficient memory for 'Hn,n,n'\n");
          qh_errexit(qh, qh_ERRmem, nullptr, nullptr);
        }
        coords = qh->feasible_point;
        value = feaspoint;
        for(i = 0; i < hulldim; ++i)
        {
          *(coords++) = *(value++);
        }
      }
      else
      {
        qh_setfeasible(qh, hulldim);
      }
      new_points = qh_sethalfspace_all(qh, dim, numpoints, points, qh->feasible_point);
      new_ismalloc = True;
      if (ismalloc) qh_free(points);
    }
    else
    {
      hulldim= dim;
      new_points= points;
      new_ismalloc= ismalloc;
    }
    qh_init_B(qh, new_points, numpoints, hulldim, new_ismalloc);
    qh_qhull(qh);
    qh_check_output(qh);

    if (outfile) qh_produce_output(qh);
    else qh_prepare_output(qh);

    if (qh->VERIFYoutput && !qh->FORCEoutput && !qh->STOPadd && !qh->STOPcone && !qh->STOPpoint)
      qh_check_points(qh);
  }
  qh->NOerrexit = True;
  return exitcode;
} /* new_qhull */

} // namespace VoronoiPlanner




