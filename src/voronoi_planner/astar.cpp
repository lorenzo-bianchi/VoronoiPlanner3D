/**
 * VoronoiPlanner - geometry.py implementation.
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
Astar::Node::Node(int idx, Node* parent, double g, double h)
: idx(idx),
  parent(parent),
  g(g),
  h(h),
  f(g + h) {}

/*  */
Astar::Astar(Result vor, Point3D start, Point3D end)
{
  this->vor = vor;
  this->dict = IndexDict(this->vor.ridges);
  this->start = this->add_ridge(start);
  this->end = this->add_ridge(end);
}

/*  */
Astar::Node* Astar::astar()
{
  auto comp = [](Node* a, Node* b) { return a->get_f() > b->get_f(); };
  std::priority_queue<Node*, std::vector<Node*>, decltype(comp)> open(comp);

  Node* start_node = this->generate_node(this->start, nullptr);
  open.push(start_node);
  std::vector<int> closed;

  while (!open.empty())
  {
    Node* current = open.top();
    open.pop();

    if (std::find(closed.begin(), closed.end(), current->get_idx()) != closed.end()) continue;

    if (this->is_goal(current->get_idx())) return current;
    else
    {
      closed.push_back(current->get_idx());
      std::vector<int> neighbors = this->dict.find(current->get_idx());
      for (int& neighbor : neighbors)
      {
        if (std::find(closed.begin(), closed.end(), neighbor) != closed.end()) continue;
        open.push(this->generate_node(neighbor, current));
      }
    }
  }

  return nullptr;
}

/*  */
std::vector<Point3D> Astar::run()
{
  Node* node = this->astar();
  std::vector<Point3D> vertices = this->vor.vertices;

  std::vector<Point3D> result;
  while (node != nullptr)
  {
    result.push_back(vertices[node->get_idx()]);
    node = node->get_parent();
  }

  if (result.empty()) throw std::invalid_argument("No path found in Astar::run");
  std::reverse(result.begin(), result.end());
  this->set_result(result);
  return result;
}

/*  */
double Astar::heuristic(int idx)
{
  Point3D cur_ver = this->vor.vertices[idx];
  Point3D goal_point = this->vor.vertices[this->end];
  return (cur_ver - goal_point).norm();
}

/*  */
Astar::Node* Astar::generate_node(int idx, Node* current)
{
  std::vector<Point3D> vertices = this->vor.vertices;

  double g;
  if (current == nullptr) g = 0;
  else
  {
    double dist = (vertices[current->get_idx()] - vertices[idx]).norm();
    g = current->get_g() + dist;
  }

  Node* node = new Node(idx, current, g, this->heuristic(idx));
  return node;
}

/*  */
bool Astar::is_goal(int idx)
{
  return idx == this->end;
}

/*  */
int Astar::add_ridge(Point3D point)
{
  // find adjacent vertices
  std::vector<int> neighbors = this->find_adjacent(point);

  // append point to vertices
  this->vor.vertices.push_back(point);
  int ver_idx = this->vor.vertices.size() - 1;

  // append ridges
  for (int& neighbor : neighbors)
  {
    RidgeVertex ridge = {neighbor, ver_idx};
    this->vor.ridges.push_back(ridge);
  }

  // insert new ridges to dictionary
  this->dict.insert(ver_idx, neighbors);
  return ver_idx;
}

/*  */
std::vector<int> Astar::find_adjacent(Point3D point)
{
  std::vector<Point3D>& vertices = this->vor.vertices;
  std::vector<int> adjacent;
  std::vector<Triangle> layers_triangles;

  std::cout << "point: " << point.transpose() << std::endl;

  /*
  // find index of altitudes vector having the same z value of point
  size_t alt_idx = 0;
  for (size_t i = 0; i < this->vor.altitudes.size(); i++)
  {
    if (abs(this->vor.altitudes[i] - point(2)) < 0.01)
    {
      alt_idx = i;
      break;
    }
  }

  // find vertex that does not intersect with ridges
  for (int i = 0; i < (int) vertices.size(); i++)
  {
    bool intersecting = false;
    for (RidgeVertex& ridge_vertex : this->vor.ridges)
    {
      if (i == ridge_vertex[0] || i == ridge_vertex[1]) continue;

      Point point2d(point(0), point(1));
      Point vertex2d(vertices[i](0), vertices[i](1));
      std::vector<Point> l1 = {point2d, vertex2d};

      Point vertex2dA(vertices[ridge_vertex[0]](0), vertices[ridge_vertex[0]](1));
      Point vertex2dB(vertices[ridge_vertex[1]](0), vertices[ridge_vertex[1]](1));
      std::vector<Point> l2 = {vertex2dA, vertex2dB};
      if (is_intersecting(l1, l2))
      {
        intersecting = true;
        break;
      }
    }

    for (Triangle& tri : this->vor.triangles[alt_idx])
  */

  int lower_idx_vec = 0;
  int upper_idx_vec = vertices.size();
  int lower_idx_rid = 0;
  int upper_idx_rid = this->vor.ridges.size();
  size_t alt_idx = 0;

  if (this->vor.altitudes.size() > 1)
  {
    // find index of altitudes vector having the same z value of point
    if (point(2) < this->vor.altitudes.front()) alt_idx = 0;
    else if (point(2) > this->vor.altitudes.back()) alt_idx = this->vor.altitudes.size()-1;
    else
    {
      for (size_t i = 0; i < this->vor.altitudes.size()-1; i++)
      {
        if (this->vor.altitudes[i] <= point(2) && point(2) <= this->vor.altitudes[i+1])
        {
          alt_idx = i;
          break;
        }
      }
    }

    // find vertex that does not intersect with ridges
    lower_idx_vec = this->vor.v_lengths[alt_idx];
    upper_idx_vec = this->vor.v_lengths[alt_idx+1];

    lower_idx_rid = this->vor.r_lengths[alt_idx];
    upper_idx_rid = this->vor.r_lengths[alt_idx+1];

    layers_triangles.insert(layers_triangles.end(), this->vor.triangles[alt_idx].begin(), this->vor.triangles[alt_idx].end());
    if (alt_idx+1 < this->vor.altitudes.size())
      layers_triangles.insert(layers_triangles.end(), this->vor.triangles[alt_idx+1].begin(), this->vor.triangles[alt_idx+1].end());
  }
  else
    layers_triangles = this->vor.triangles[0];

  for (int i = lower_idx_vec; i < upper_idx_vec; i++)
  {
    bool intersecting = false;
    for (int j = lower_idx_rid; j < upper_idx_rid; j++)
    {
      RidgeVertex ridge_vertex = this->vor.ridges[j];
      if (i == ridge_vertex[0] || i == ridge_vertex[1]) continue;

      Point point2d(point(0), point(1));
      Point vertex2d(vertices[i](0), vertices[i](1));
      std::vector<Point> l1 = {point2d, vertex2d};

      Point vertex2dA(vertices[ridge_vertex[0]](0), vertices[ridge_vertex[0]](1));
      Point vertex2dB(vertices[ridge_vertex[1]](0), vertices[ridge_vertex[1]](1));
      std::vector<Point> l2 = {vertex2dA, vertex2dB};
      if (is_intersecting(l1, l2))
      {
        intersecting = true;
        break;
      }
    }

    for (Triangle& tri : layers_triangles)
    {
      auto points = tri.get_points();

      Point point1 = points[0];
      Point point2 = points[1];
      Point point3 = points[2];

      Point point2d(point(0), point(1));
      Point vertex2d(vertices[i](0), vertices[i](1));
      std::vector<Point> l1 = {point2d, vertex2d};

      double k = 0.2; // TODO: line_increase_;
      double norm21 = (point2 - point1).norm();
      double norm32 = (point3 - point2).norm();
      double norm13 = (point1 - point3).norm();
      std::vector<Point> l2 = {point1 - k * (point2 - point1) / norm21,
                               point2 + k * (point2 - point1) / norm21};
      std::vector<Point> l3 = {point2 - k * (point3 - point2) / norm32,
                               point3 + k * (point3 - point2) / norm32};
      std::vector<Point> l4 = {point3 - k * (point1 - point3) / norm13,
                               point1 + k * (point1 - point3) / norm13};

      if (is_intersecting(l1, l2) || is_intersecting(l1, l3) || is_intersecting(l1, l4))
      {
        intersecting = true;
        break;
      }
    }

    if (!intersecting)
    {
      std::cout << "i: " << i << std::endl;
      adjacent.push_back(i);
    }
  }
  return adjacent;
}

/*  */
void Astar::generate_plot()
{
//   std::vector<double> X, Y, VX, VY;
//   for (auto vector : this->vor.points)
//   {
//     X.push_back(vector[0]);
//     Y.push_back(vector[1]);
//   }
//   for (auto vector : this->vor.vertices)
//   {
//     VX.push_back(vector[0]);
//     VY.push_back(vector[1]);
//   }

//   std::vector<std::vector<Point>> finite_segments, infinite_segments;
//   int rv_size = this->vor.ridges.size();
//   for (int i = 0; i < rv_size; i++)
//   {
//     RidgeVertex simplex = this->vor.ridges[i];

//     // check if all simplex values are >= 0
//     bool check = true;
//     for (auto ele : simplex)
//     {
//       if (ele < 0)
//       {
//         check = false;
//         break;
//       }
//     }
//     if (check)
//     {
//       int idx_x = simplex[0];
//       int idx_y = simplex[1];

//       if (idx_x > (int) this->vor.vertices.size() || idx_y > (int) this->vor.vertices.size())
//       {
//         throw std::invalid_argument("Error in generate_plot()");
//       }

//       Point p1 = this->vor.vertices[idx_x];
//       Point p2 = this->vor.vertices[idx_y];
//       finite_segments.push_back({p1, p2});
//     }
//     else
//     {
//       // infinite_segments
//     }
//   }
//   plt::figure_size(640, 480);

//   // plot finite_segments
//   std::vector<double> x, y;
//   for (auto& segment : finite_segments)
//   {
//     x = {segment[0][0], segment[1][0]};
//     y = {segment[0][1], segment[1][1]};
//     plt::plot(x, y, "b");
//   }

// std::cout << "this->result.size(): " << this->result.size() << std::endl;
//   for (size_t i = 0; i < this->result.size()-1; i++)
//   {
//     x = {result[i][0], result[i+1][0]};
//     y = {result[i][1], result[i+1][1]};
//     plt::plot(x, y, "r");
//   }

//   plt::plot(X, Y, "b.");
//   plt::plot(VX, VY, "yo");

//   plt::legend();
//   plt::set_aspect_equal();
//   plt::show();
}

} // namespace VoronoiPlanner
