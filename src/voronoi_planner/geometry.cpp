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
  // General functions
  /*  */
  double total_distance(std::vector<Point>& path)
  {
    double dist = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++)
    {
      dist += (path[i] - path[i + 1]).norm();
    }
    return dist;
  }

  /*  */
  double distance_between_line_point(std::vector<Point>& line, Point& point)
  {
    Eigen::Vector2d v1 = line[1] - line[0];
    Eigen::Vector2d v2 = point - line[0];
    double l1 = v1.norm();
    double l2 = v2.norm();

    double dot = v1.dot(v2);
    if (dot > l1 * l1 || dot < 0.0)
    {
      return std::min(l2, (point - line[1]).norm());
    }

    double angle = acos(dot / (l1 * l2));
    return sin(angle) * l2;
  }

  /*  */
  double radian(Eigen::Vector2d& v1, Eigen::Vector2d& v2)
  {
    v1.normalize();
    v2.normalize();
    double dot = v1.dot(v2);
    if (dot > 1.0) dot = 1.0;
    return acos(dot);
  }

  /*  */
  int find_closest(Chain vec, int elem)
  {
    for (int i = 0; i < (int) vec.size(); i++)
    {
      if (vec[i] > elem) return i;
    }
    return vec.size();
  }

  /*  */
  int counter_clockwise(Point& point1, Point& point2, Point& point3)
  {
    Eigen::Vector2d v1 = point2 - point1;
    Eigen::Vector2d v2 = point3 - point1;

    // Compute cross product between v1 and v2
    double cross = v1[0] * v2[1] - v1[1] * v2[0];
    double thr = 0.000001;
    if (cross > thr) return 1;
    else if (abs(cross) <= thr) return 0;

    return -1;
  }

  /*  */
  bool is_intersecting(std::vector<Point>& line1, std::vector<Point>& line2)
  {
    int ccw1 = counter_clockwise(line1[0], line1[1], line2[0]);
    int ccw2 = counter_clockwise(line1[0], line1[1], line2[1]);

    if (ccw1 * ccw2 < 0.0)
    {
      int ccw1 = counter_clockwise(line2[0], line2[1], line1[0]);
      int ccw2 = counter_clockwise(line2[0], line2[1], line1[1]);

      if (ccw1 * ccw2 < 0.0) return true;
    }
    return false;
  }

  /*  */
  std::vector<Triangle> triangulation(Polygon& polygon)
  {
    std::vector<Triangle> triangles = {};
    earclip(polygon, triangles);
    return triangles;
  }

  // Line class implementation
  /*  */
  Line::Line() {}

  /*  */
  Line::Line(std::vector<Point> inputPoints) : points(inputPoints)
  {
    if (points.size() != 2) throw std::invalid_argument("Line must have 2 points.");
  }

  /*  */
  std::vector<Point> Line::generate_line()
  {
    return generate_line_base(points[0], points[1]);
  }

  /*  */
  std::vector<Point> Line::generate_line_base(Point& point1, Point& point2)
  {
    std::vector<Point> result = {};

    if (point1[0] != point2[0])
    {
      auto temp = generate_line_base(point1, point2, 0);
      result.insert(result.end(), temp.begin(), temp.end());
    }
    else if (point1[1] != point2[1])
    {
      auto temp = generate_line_base(point1, point2, 1);
      result.insert(result.end(), temp.begin(), temp.end());
    }
    else
    {
      std::cout << "Identical point has been detected: [" << point1[0] << ", " << point1[1] << "]" << std::endl;
    }
    return result;
  }

  /*  */
  std::vector<Point> Line::generate_line_base(Point& point1, Point& point2, int axis)
  {
    std::vector<Point> result;

    if (point1[axis] > point2[axis])
    {
      Point temp = point1;
      point1 = point2;
      point2 = temp;
    }

    double dis = this->distance(point1, point2);

    Eigen::Vector2d stepVec = (point2 - point1) / dis;
    stepVec *= this->point_distance;

    Point current = point1;
    while (current[axis] <= point2[axis])
    {
      result.push_back(current);
      current += stepVec;
    }
    return result;
  }

  /*  */
  bool Line::is_intersecting_class(Line& otherLine)
  {
    return is_intersecting(points, otherLine.points);
  }

  double Line::distance(Point& point1, Point& point2)
  {
    return (point1 - point2).norm();
  }

  // Triangle class implementation
  Triangle::Triangle(std::vector<Point> inputPoints)
  {
    points = inputPoints;
    if (points.size() != 3) throw std::invalid_argument("Triangle must have 3 points.");
  }

  /*  */
  std::vector<Point> Triangle::generate_line()
  {
    std::vector<Point> result;

    // loop for every line in Triangle
    for (size_t i = 0; i < this->points.size(); i++)
    {
      Point p1 = this->points[i];
      Point p2 = this->points[i + 1 < this->points.size() ? i + 1 : 0];
      auto v = generate_line_base(p1, p2);
      result.insert(result.end(), v.begin(), v.end());
    }

    return result;
  }

  /*  */
  bool Triangle::is_in_polygon(Point& point)
  {
    // if point is close enough, filter out
    if (test_distance_tresh(this->points, point, this->distance_thresh))
    {
      return true;
    }
    return test_point_convex(this->points, point);
  }

  /*  */
  bool Triangle::test_distance_tresh(
    std::vector<Point>& points,
    Point& test_point,
    double distance_thresh)
  {
    for (auto& point : points)
    {
      double dist = (test_point - point).norm();
      if (dist < distance_thresh) return true;
    }
    return false;
  }

  /*  */
  bool Triangle::test_point_convex(std::vector<Point>& points, Point& test_point)
  {
    for (size_t i = 0; i < 3; i++)
    {
      int ccw1 = counter_clockwise(points[i], points[(i + 1) % 3], test_point);
      int ccw2 = counter_clockwise(points[i], test_point, points[(i + 2) % 3]);

      if (ccw1 * ccw2 <= 0.0) return false;
    }
    return true;
  }

} // namespace VoronoiPlanner
