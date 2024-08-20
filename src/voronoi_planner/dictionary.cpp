/**
 * VoronoiPlanner - dictionary.py implementation.
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
IndexDict::IndexDict(RidgeVertices& vec)
{
  Dict dict = this->generate(vec, false);
  Dict rdict = this->generate(vec, true);

  for (auto& entry : rdict)
  {
    int key = entry.first;
    Chain value = entry.second;

    if (contains(dict, key))
    {
      for (int& ele : value)
      {
        if (std::find(dict[key].begin(), dict[key].end(), ele) == dict[key].end())
        {
          dict[key].push_back(ele);
        }
      }
    }
    else dict[key] = value;
  }

  this->dict = dict;
}

/*  */
void IndexDict::insert(int key, Chain& value)
{
  if (contains(this->dict, key))
  {
    // print dict and key
    for (auto& entry : this->dict)
    {
      std::cout << entry.first << std::endl;
    }
    std::cout << key << std::endl;

    throw std::invalid_argument("Key already exists");
  }

  this->dict[key] = value;

  for (int& ele : value)
  {
    if (contains(this->dict, ele))
    {
      this->dict[ele].push_back(key);
    }
    else
    {
      this->dict[ele] = {key};
    }
  }
}

/*  */
Dict IndexDict::generate(RidgeVertices& vec, bool reverse)
{
  int idx = reverse ? 1 : 0;

  RidgeVertices sorted = vec;
  std::stable_sort(
    sorted.begin(), sorted.end(), [idx](const RidgeVertex& a, const RidgeVertex& b)
    {
      return a[idx] < b[idx];
    });

  Chain organized;
  std::vector<int> key;
  Chains value;

  int current = sorted[0][idx];

  for (auto& ele : sorted)
  {
    if (ele[idx] != current)
    {
      value.push_back(organized);
      key.push_back(current);
      organized.clear();
      current = ele[idx];
    }
    organized.push_back(reverse ? ele[0] : ele[1]);
  }

  value.push_back(organized);
  key.push_back(current);

  Dict result;
  for (size_t i = 0; i < key.size(); i++)
  {
    result[key[i]] = value[i];
  }

  return result;
}

/*  */
bool IndexDict::contains(Dict& dict, int key)
{
  return dict.find(key) != dict.end();
}

/*  */
bool IndexDict::contains(int key)
{
  return contains(this->dict, key);
}

/*  */
Chain IndexDict::find(int key)
{
  if (contains(key)) return this->dict.at(key);
  return {};
}

/*  */
std::vector<std::pair<int, Chain>> IndexDict::items()
{
  std::vector<std::pair<int, Chain>> result;
  for (auto& entry : this->dict)
  {
    result.push_back(entry);
  }
  return result;
}

} // namespace VoronoiPlanner
