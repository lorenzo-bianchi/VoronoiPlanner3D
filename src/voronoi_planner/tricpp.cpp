/**
 * VoronoiPlanner Tricpp functions.
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
  bool is_clockwise(Polygon& polygon)
  {
    double s = 0;
    int polygonCount = polygon.size();
    for (int i = 0; i < polygonCount; ++i)
    {
      Point & point = polygon[i];
      Point & point2 = polygon[(i + 1) % polygonCount];
      s += (point2[0] - point[0]) * (point2[1] + point[1]);
    }
    return s > 0;
  }

  double triangle_sum(double x1, double y1, double x2, double y2, double x3, double y3)
  {
    return x1 * (y3 - y2) + x2 * (y1 - y3) + x3 * (y2 - y1);
  }

  bool is_convex(Point& prev, Point& point, Point& next)
  {
    return triangle_sum(prev[0], prev[1], point[0], point[1], next[0], next[1]) < 0;
  }

  double triangle_area(Point& p1, Point& p2, Point& p3)
  {
    return std::abs(
      (p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])) / 2.0);
  }

  bool is_point_inside(Point& p, Point& a, Point& b, Point& c)
  {
    double area = triangle_area(a, b, c);
    double area1 = triangle_area(p, b, c);
    double area2 = triangle_area(p, a, c);
    double area3 = triangle_area(p, a, b);
    double areaDiff = std::abs(area - (area1 + area2 + area3));
    return areaDiff < EPSILON;
  }

  bool contains_no_points(Point& p1, Point& p2, Point& p3, Polygon& polygon)
  {
    for (Point& pn : polygon)
    {
      if (pn == p1 || pn == p2 || pn == p3)
      {
        continue;
      }
      else if (is_point_inside(pn, p1, p2, p3))
      {
        return false;
      }
    }
    return true;
  }

  bool is_ear(Point& p1, Point& p2, Point& p3, Polygon& polygon)
  {
    return contains_no_points(p1, p2, p3, polygon) &&
           is_convex(p1, p2, p3) &&
           triangle_area(p1, p2, p3) > 0;
  }

  void earclip(Polygon& polygon, std::vector<Triangle>& triangles)
  {
    Polygon earVertex = {};
    Polygon polygonCopy = polygon;
    if (is_clockwise(polygon))
    {
      std::reverse(polygonCopy.begin(), polygonCopy.end());
    }

    int pointCount = polygonCopy.size();
    for (int i = 0; i < pointCount; i++)
    {
      int prevIndex = (i - 1 + pointCount) % pointCount;
      Point& prevPoint = polygonCopy[prevIndex];
      Point& point = polygonCopy[i];
      int nextIndex = (i + 1) % pointCount;
      Point& nextPoint = polygonCopy[nextIndex];

      if (is_ear(prevPoint, point, nextPoint, polygonCopy))
      {
        earVertex.push_back(point);
      }
    }

    while (!earVertex.empty() && pointCount >= 3)
    {
      Point ear = earVertex.front();
      earVertex.erase(earVertex.begin());

      auto it = std::find(polygonCopy.begin(), polygonCopy.end(), ear);
      int i = std::distance(polygonCopy.begin(), it);

      int prevIndex = (i - 1 + pointCount) % pointCount;
      Point prevPoint = polygonCopy[prevIndex];

      int nextIndex = (i + 1) % pointCount;
      Point nextPoint = polygonCopy[nextIndex];

      polygonCopy.erase(it);
      if (i == 0) prevIndex--;

      pointCount--;
      triangles.push_back(Triangle({prevPoint, ear, nextPoint}));

      if (pointCount > 3)
      {
        int prevPrevIndex = (prevIndex - 1 + pointCount) % pointCount;
        Point prevPrevPoint = polygonCopy[prevPrevIndex];

        int nextNextIndex = (i + 1) % pointCount;
        Point nextNextPoint = polygonCopy[nextNextIndex];

        std::vector<std::tuple<Point, Point, Point, Polygon>> groups =
        {
          {prevPrevPoint, prevPoint, nextPoint, polygonCopy},
          {prevPoint, nextPoint, nextNextPoint, polygonCopy}
        };

        for (auto& group : groups)
        {
          Point p = std::get<1>(group);
          auto earIt = std::find(earVertex.begin(), earVertex.end(), p);
          if (is_ear(std::get<0>(group), p, std::get<2>(group), std::get<3>(group)))
          {
            if (earIt == earVertex.end()) earVertex.push_back(p);
          }
          else if (earIt != earVertex.end())
          {
            earVertex.erase(earIt);
          }
        }
      }
    }
  }

  double calculate_total_area(std::vector<Triangle>& triangles)
  {
    double result = 0.0;
    for (auto& triangle : triangles)
    {
      double sides[3];
      for (int i = 0; i < 3; i++)
      {
        int nextIndex = (i + 1) % 3;
        std::vector<Point> points = triangle.get_points();
        Point& pt = (i == 0) ? points[0] : ((i == 1) ? points[1] : points[2]);
        Point& pt2 =
          (nextIndex == 0) ? points[0] : ((nextIndex == 1) ? points[1] : points[2]);
        double side = (pt2 - pt).norm();
        sides[i] = side;
      }

      std::sort(std::begin(sides), std::end(sides));
      double a = sides[0];
      double b = sides[1];
      double c = sides[2];

      double area = 0.25 *
        std::sqrt(std::abs((a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c))));
      result += area;
    }

    return result;
  }
} // namespace VoronoiPlanner
