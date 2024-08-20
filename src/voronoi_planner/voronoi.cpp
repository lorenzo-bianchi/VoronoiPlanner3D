/**
 * VoronoiPlanner - voronoi.py implementation.
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
/*  */
GeneralizedVoronoi::GeneralizedVoronoi() {}

/*  */
void GeneralizedVoronoi::add_point(Point& point)
{
  this->points.push_back(point);
}

/*  */
void GeneralizedVoronoi::add_points(std::vector<Point>& new_points)
{
  this->points.insert(this->points.end(), new_points.begin(), new_points.end());
}

/*  */
void GeneralizedVoronoi::add_line(Line& line)
{
  this->lines.push_back(line);

  auto temp = line.generate_line();
  this->line_lined_points.insert(this->line_lined_points.end(), temp.begin(), temp.end());

  auto pts = line.get_points();
  this->line_points.insert(this->line_points.end(), pts.begin(), pts.end());
}

/*  */
void GeneralizedVoronoi::add_lines(std::vector<Line>& new_lines)
{
  for (auto& line : new_lines)
  {
    this->add_line(line);
  }
}

/*  */
void GeneralizedVoronoi::add_triangle(Triangle& triangle)
{
  this->triangles.push_back(triangle);

  auto temp = triangle.generate_line();
  this->triangle_lined_points.insert(this->triangle_lined_points.end(), temp.begin(), temp.end());

  auto tri = triangle.get_points();
  this->triangle_points.insert(this->triangle_points.end(), tri.begin(), tri.end());
}

/*  */
void GeneralizedVoronoi::add_triangles(std::vector<Triangle>& new_triangles)
{
  for (auto& triangle : new_triangles)
  {
    this->add_triangle(triangle);
  }
}

/*  */
void GeneralizedVoronoi::add_boundary(Line& boundary)
{
  this->boundaries.push_back(boundary);

  auto temp = boundary.generate_line();
  this->boundary_lined_points.insert(this->boundary_lined_points.end(), temp.begin(), temp.end());

  auto bound = boundary.get_points();
  this->boundary_points.insert(this->boundary_points.end(), bound.begin(), bound.end());
}

/*  */
void GeneralizedVoronoi::add_boundaries(std::vector<Line>& new_boundaries)
{
  for (auto& boundary : new_boundaries)
  {
    this->add_boundary(boundary);
  }
}

/*  */
void GeneralizedVoronoi::add_polygons(Polygons& polygons)
{
  for (auto& polygon : polygons)
  {
    Point center = {0, 0};
    for (auto& vertex : polygon)
    {
      center += vertex;
    }
    center /= polygon.size();
    this->polygons.push_back({center, polygon});

    this->add_polygon(polygon);
  }
}

/*  */
void GeneralizedVoronoi::add_polygon(Polygon& polygon)
{
  auto triangles = triangulation(polygon);
  for (auto& vertices : triangles)
  {
    Triangle tri = Triangle(vertices);
    this->add_triangle(tri);
  }
}

/*  */
Chain GeneralizedVoronoi::vertices_in_polygon()
{
  Chain in_polygon = {};

  for (size_t i = 0; i < this->vor.vertices.size(); i++)
  {
    for (auto& tri : this->triangles)
    {
      if (tri.is_in_polygon(this->vor.vertices[i]))
      {
        in_polygon.push_back(i);
        break;
      }
    }
  }

  return in_polygon;
}

/*  */
void GeneralizedVoronoi::run_non_optimized()
{
  std::vector<Point> all_points = {};
  all_points.insert(all_points.end(), boundary_lined_points.begin(), boundary_lined_points.end());
  all_points.insert(all_points.end(), triangle_lined_points.begin(), triangle_lined_points.end());
  all_points.insert(all_points.end(), line_lined_points.begin(), line_lined_points.end());
  all_points.insert(all_points.end(), points.begin(), points.end());

  this->run_voronoi(all_points);

  // calculate unreachable vertices and ridges
  Chain unreachable_vertices = this->vertices_in_polygon();
  Chain ridge_to_delete = this->ridges_to_delete(unreachable_vertices);

  // delete unreachable vertices and ridges
  this->delete_vertex(unreachable_vertices);
  this->delete_ridge(ridge_to_delete);

  // reorganize ridge vertices
  this->reorganize_ridge(unreachable_vertices);
}

/*  */
void GeneralizedVoronoi::run_optimized()
{
  this->run_non_optimized();

  while (true)
  {
    if (!this->optimize_line()) break;
    this->delete_unfinished();
  }
}

/*  */
void GeneralizedVoronoi::delete_unfinished()
{
  // regenerate chain by optimized value
  this->chains = this->generate_chains();

  // calculate unfinished vertices and ridges
  Chain unfinished_vertices = this->unfinished_vertices();
  Chain ridge_to_delete = this->ridges_to_delete(unfinished_vertices);

  // delete unfinished vertices and ridges
  this->delete_vertex(unfinished_vertices);
  this->delete_ridge(ridge_to_delete);
  this->reorganize_ridge(unfinished_vertices);
}

/*  */
Chain GeneralizedVoronoi::unfinished_vertices()
{
  IndexDict dict = IndexDict(this->vor.ridge_vertices);
  Chain unfinished = {};

  for (auto& entry : dict.items())
  {
    if (entry.second.size() == 1)
    {
      unfinished.push_back(entry.first);
    }
  }

  Chain chain_vertices = {};
  for (auto& ele : unfinished)
  {
    for (auto& chain : this->chains)
    {
      if (ele == chain[0])
      {
        chain_vertices.insert(chain_vertices.end(), chain.begin(), chain.end()-1);
        break;
      }
      else if (ele == chain.back())
      {
        chain_vertices.insert(chain_vertices.end(), chain.begin()+1, chain.end());
        break;
      }
    }
  }

  std::stable_sort(chain_vertices.begin(), chain_vertices.end());
  return chain_vertices;
}

/*  */
Chain GeneralizedVoronoi::ridges_to_delete(Chain& vertex_vec)
{
  std::vector<int> to_delete = {};
  VertexChain vertices = this->vor.vertices;

  for (size_t i = 0; i < this->vor.ridge_vertices.size(); i++)
  {
    RidgeVertex rv = this->vor.ridge_vertices[i];

    // if ridge heads outside, delete ridge
    if (rv[0] == -1 || rv[1] == -1)
    {
      to_delete.push_back(i);
      continue;
    }

    // if ridge contains deleted vertex, delete ridge
    bool deleted = false;
    for (auto& ver : vertex_vec)
    {
      if (ver == rv[0] || ver == rv[1])
      {
        to_delete.push_back(i);
        deleted = true;
        break;
      }
    }
    if (deleted) continue;

    // if ridge intersects with line, delete ridge
    for (auto& line : this->lines)
    {
      Line l2 = Line({vertices[rv[0]], vertices[rv[1]]});
      if (line.is_intersecting_class(l2))
      {
        to_delete.push_back(i);
        break;
      }
    }
  }

  return to_delete;
}

/*  */
void GeneralizedVoronoi::delete_vertex(Chain to_delete)
{
  std::sort(to_delete.rbegin(), to_delete.rend());
  for (int& idx : to_delete)
  {
    this->vor.vertices.erase(this->vor.vertices.begin() + idx);
  }
}

/*  */
void GeneralizedVoronoi::delete_ridge(Chain to_delete)
{
  std::sort(to_delete.rbegin(), to_delete.rend());
  for (int& idx : to_delete)
  {
    this->vor.ridge_vertices.erase(this->vor.ridge_vertices.begin() + idx);
  }
}

/*  */
void GeneralizedVoronoi::reorganize_ridge(Chain& deleted_vertices)
{
  for (RidgeVertex& rv : this->vor.ridge_vertices)
  {
    int rv0 = rv[0];
    int rv1 = rv[1];
    int rv0i = find_closest(deleted_vertices, rv0);
    int rv1i = find_closest(deleted_vertices, rv1);
    rv = RidgeVertex({rv0 - rv0i, rv1 - rv1i});
  }
}

/*  */
void GeneralizedVoronoi::run(run_type type, bool plot, double altitude, Result& results)
{
  switch (type)
  {
    case run_type::non_optimized:
      this->run_non_optimized();
      break;
    case run_type::optimized:
      this->run_optimized();
      break;
  }

  this->generate_result(altitude, results);

  if (plot) this->generate_plot();
}

/*  */
void GeneralizedVoronoi::generate_plot()
{
  // std::vector<double> X, Y, VX, VY;
  // for (auto vector : this->vor.points)
  // {
  //   X.push_back(vector[0]);
  //   Y.push_back(vector[1]);
  // }
  // for (auto vector : this->vor.vertices)
  // {
  //   VX.push_back(vector[0]);
  //   VY.push_back(vector[1]);
  // }

  // std::vector<std::vector<Point>> finite_segments, infinite_segments;
  // int rp_size = this->vor.ridge_points.size();
  // int rv_size = this->vor.ridge_vertices.size();
  // for (int i = 0; i < std::min(rp_size, rv_size); i++)
  // {
  //   RidgeVertex simplex = this->vor.ridge_vertices[i];

  //   // check if all simplex values are >= 0
  //   bool check = true;
  //   for (auto ele : simplex)
  //   {
  //     if (ele < 0)
  //     {
  //       check = false;
  //       break;
  //     }
  //   }
  //   if (check)
  //   {
  //     int idx_x = simplex[0];
  //     int idx_y = simplex[1];

  //     if (idx_x > (int) this->vor.vertices.size() || idx_y > (int) this->vor.vertices.size())
  //     {
  //       throw std::invalid_argument("Error in generate_plot()");
  //     }

  //     Point p1 = this->vor.vertices[idx_x];
  //     Point p2 = this->vor.vertices[idx_y];
  //     finite_segments.push_back({p1, p2});
  //   }
  //   else
  //   {
  //     // infinite_segments
  //   }
  // }

  // plt::figure_size(640, 480);
  // plt::plot(X, Y, "b.");
  // plt::plot(VX, VY, "yo");

  // // plot finite_segments
  // std::vector<double> x, y;
  // for (auto& segment : finite_segments)
  // {
  //   x = {segment[0][0], segment[1][0]};
  //   y = {segment[0][1], segment[1][1]};
  //   plt::plot(x, y, "k");
  // }
  // plt::legend();
  // plt::set_aspect_equal();
  // plt::show();
}

/*  */
bool GeneralizedVoronoi::optimize_line()
{
  this->chains = this->generate_chains();
  if (this->chains.size() == 0) return false;

  VertexChains vertex_chains = this->generate_vertex_chains(this->chains);

  if (this->chains.empty()) return false;

  VertexChains optimized_chains = this->optimize_line_base(vertex_chains);
  this->regenerate_voronoi(optimized_chains);

  return true;
}

/*  */
void GeneralizedVoronoi::regenerate_voronoi(VertexChains& chains)
{
  VertexChain vertices = {};
  RidgeVertices ridge_vertices = {};

  for (auto& chain : chains)
  {
    if (std::find(vertices.begin(), vertices.end(), chain[0]) == vertices.end())
    {
      vertices.push_back(chain[0]);
    }

    for (size_t i = 0; i < chain.size()-1; i++)
    {
      int index = std::find(vertices.begin(), vertices.end(), chain[i]) - vertices.begin();
      RidgeVertex idx = {index, -1};

      if (std::find(vertices.begin(), vertices.end(), chain[i+1]) == vertices.end())
      {
        vertices.push_back(chain[i+1]);
        idx[1] = vertices.size()-1;
      }
      else
      {
        index = std::find(vertices.begin(), vertices.end(), chain[i+1]) - vertices.begin();
        idx[1] = index;
      }

      ridge_vertices.push_back(idx);
    }
  }

  this->vor.vertices = vertices;
  this->vor.ridge_vertices = ridge_vertices;
}

/*  */
VertexChains GeneralizedVoronoi::optimize_line_base(VertexChains& chains)
{
  VertexChains optimized_chains = {};
  for (auto& chain : chains)
  {
    VertexChain out = {};
    ramer_douglas_peucker(chain, rdp_epsilon, out);
    optimized_chains.push_back(out);
  }
  return optimized_chains;
}

/*  */
void GeneralizedVoronoi::run_voronoi(std::vector<Point>& points)
{
  this->vor = Voronoi(points);
}

/*  */
void GeneralizedVoronoi::generate_result(double altitude, Result& results)
{
  // transform vertices from 2d to 3d
  std::vector<Point3D> vertices(this->vor.vertices.size());
  for (size_t i = 0; i < this->vor.vertices.size(); i++)
    vertices[i] = {this->vor.vertices[i][0], this->vor.vertices[i][1], altitude};
  results.vertices.insert(results.vertices.end(), vertices.begin(), vertices.end());

  for (size_t i = 0; i < this->vor.ridge_vertices.size(); i++)
    this->vor.ridge_vertices[i] += Eigen::Vector2i(results.v_lengths.back(), results.v_lengths.back());
  results.ridges.insert(results.ridges.end(), this->vor.ridge_vertices.begin(), this->vor.ridge_vertices.end());

  results.r_lengths.push_back(results.r_lengths.back() + this->vor.ridge_vertices.size());
  results.v_lengths.push_back(results.v_lengths.back() + this->vor.vertices.size());

  results.points.push_back(this->vor.points);
  results.triangles.push_back(this->triangles);

  results.altitudes.push_back(altitude);
}

/*  */
Chains GeneralizedVoronoi::generate_chains()
{
  IndexDict dict = IndexDict(this->vor.ridge_vertices);

  // ignition value must be dead end point (which has only 1 neighbor)
  int ignition_idx = -1;
  for (std::pair<int, Chain>& entry : dict.items())
  {
    int key = entry.first;
    Chain value = entry.second;

    if (value.size() == 1)
    {
      ignition_idx = key;
      break;
    }
  }

  // voronoi diagram with no dead end point cannot be optimized
  if (ignition_idx == -1) return {};

  // generate chains
  Chain feature_point = {};
  Chains chains = {};
  ChainStart start_point = {};
  start_point.push_back({-1, ignition_idx});

  while (start_point.size() > 0)
  {
    Chain temp = this->generate_chain(dict, start_point, feature_point);
    chains.push_back(temp);
  }

  return chains;
}

/*  */
Chain GeneralizedVoronoi::generate_chain(IndexDict& dict, ChainStart& start, Chain& feature)
{
  Chain chain = {};

  // get new starting point
  ChainIdx idx = start.back();
  start.pop_back();

  // case of dead end point
  if (idx[0] != -1) chain.push_back(idx[0]);

  // ignite chain
  ChainStart new_start = this->chain(dict, idx, chain, feature);

  // add chain start and end to feature points
  feature.push_back(chain.front());
  feature.push_back(chain.back());

  // add new starting points to queue
  for (auto& ele : new_start)
  {
    start.push_back(ele);
  }

  return chain;
}

/*  */
ChainStart GeneralizedVoronoi::chain(IndexDict& dict, ChainIdx& idx, Chain& chain, Chain& feature)
{
  // append current point to chain
  chain.push_back(idx[1]);

  // search neighbor on index dictionary
  Chain neighbor = dict.find(idx[1]);
  int neighbor_count = neighbor.size();

  // visited is selected base on feature point(diverging point) and previous point
  // append idx[0] to feature and assign to visited
  Chain visited = feature;
  visited.push_back(idx[0]);

  // case 1, dead end point
  if (neighbor_count == 1)
  {
    if (neighbor[0] == idx[0]) return {};
    ChainIdx new_idx = {idx[1], neighbor[0]};
    return this->chain(dict, new_idx, chain, feature);
  }

  // case 2, middle of line
  else if (neighbor_count == 2)
  {
    std::vector<bool> has_visited = {false, false};
    // check if neighbor[0] is in visited
    bool check1 = std::find(visited.begin(), visited.end(), neighbor[0]) != visited.end();
    bool check2 = std::find(visited.begin(), visited.end(), neighbor[1]) != visited.end();

    if (check1) has_visited[0] = true;
    if (check2) has_visited[1] = true;

    int next_idx;
    // if both neighbor is visited, it's end of line
    if (has_visited[0] && has_visited[1])
    {
      if (idx[0] == neighbor[0]) chain.push_back(neighbor[1]);
      else if (idx[0] == neighbor[1]) chain.push_back(neighbor[0]);
      return {};
    }

    // prevent going back
    else if (has_visited[0]) next_idx = 1;
    else if (has_visited[1]) next_idx = 0;
    else throw std::invalid_argument("Error in chain generation");

    // find next vertex
    ChainIdx new_idx = {idx[1], neighbor[next_idx]};
    return this->chain(dict, new_idx, chain, feature);
  }

  // case more than 2, diverging point
  // line must end on diverging point
  // new starting points must be added for full construction
  ChainStart new_start_points = {};
  for (int i = 0; i < neighbor_count; i++)
  {
    bool check = std::find(visited.begin(), visited.end(), neighbor[i]) != visited.end();
    if (check) continue;
    new_start_points.push_back({idx[1], neighbor[i]});
  }
  return new_start_points;
}

/*  */
std::vector<std::vector<Point>> GeneralizedVoronoi::generate_vertex_chains(Chains& chains)
{
  std::vector<std::vector<Point>> vertex_chains = {};
  for (auto& chain : chains)
  {
    std::vector<Point> vertex_chain = {};
    for (auto& ele : chain)
    {
      vertex_chain.push_back(this->vor.vertices[ele]);
    }
    vertex_chains.push_back(vertex_chain);
  }
  return vertex_chains;
}

} // namespace VoronoiPlanner