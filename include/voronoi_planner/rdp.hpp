/**
 * VoronoiPlanner ramer_douglas_peucker functions.
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

#ifndef RDP_HPP
#define RDP_HPP

#include <Eigen/Dense>

namespace VoronoiPlanner
{
// RDP
template <class T>
double perpendicular_distance(T& pt, T& lineStart, T& lineEnd)
{
  T dx = lineEnd - lineStart;
  dx.normalize();

  T pv = pt - lineStart;

  // Get dot product
  double pvdot = dx.dot(pv);

  // Scale line direction vector
  T ds = pvdot * dx;

  // Subtract from pv
  T a = pv - ds;

  return a.norm();
}

template <class T>
void ramer_douglas_peucker(std::vector<T>& pointList, double epsilon, std::vector<T>& out)
{
  if (pointList.size() < 2) throw std::invalid_argument("Not enough points to simplify");

  // Find the point with maximum distance from line between start and end
  double dmax = 0.0;
  size_t index = 0;
  size_t end = pointList.size()-1;
  for (size_t i = 1; i < end; i++)
  {
    double d = perpendicular_distance(pointList[i], pointList[0], pointList[end]);
    if (d > dmax)
    {
      index = i;
      dmax = d;
    }
  }

  // If max distance is greater than epsilon, recursively simplify
  if (dmax > epsilon)
  {
    // Recursive call
    std::vector<T> recResults1;
    std::vector<T> recResults2;
    std::vector<T> firstLine(pointList.begin(), pointList.begin()+index+1);
    std::vector<T> lastLine(pointList.begin()+index, pointList.end());
    ramer_douglas_peucker(firstLine, epsilon, recResults1);
    ramer_douglas_peucker(lastLine, epsilon, recResults2);

    // Build the result list
    out.assign(recResults1.begin(), recResults1.end()-1);
    out.insert(out.end(), recResults2.begin(), recResults2.end());
    if (out.size() < 2) throw std::runtime_error("Problem assembling output");
  }
  else
  {
    // Return start and end points
    out.clear();
    out.push_back(pointList[0]);
    out.push_back(pointList[end]);
  }
}

} // namespace VoronoiPlanner

#endif // VORONOI_PLANNER_HPP