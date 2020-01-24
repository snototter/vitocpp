#ifndef __VCP_UTILS_SORT_UTILS_H__
#define __VCP_UTILS_SORT_UTILS_H__

#include <vector>
#include <algorithm>
#include <utility>
#include <exception>
#include "vcp_error.h"


namespace vcp
{
namespace utils
{

/** @brief Extract the keys from a map container. */
template <typename M>
std::vector<typename M::key_type> GetMapKeys(const M &map)
{
  std::vector<typename M::key_type> vec;
  vec.reserve(map.size());
  for (typename M::const_iterator it = map.begin(); it != map.end(); ++it)
    vec.push_back(it->first);
  return vec;
}


/** @brief Sort comparator to sort ascending if operator< is implemented. */
template <typename _T>
bool CmpAsc(const _T &a, const _T &b)
{
  return a < b;
}


/** @brief Sort comparator to sort descending if operator< is implemented. */
template <typename _T>
bool CmpDesc(const _T &a, const _T &b)
{
  return b < a;
}


/** @brief Utility class to get the ordering (i.e. the sorted indices) of a vector. */
template <typename _T>
class Ordering
{
public:
  Ordering(const std::vector<_T> &data) : data_(data), cmp_(CmpAsc<_T>) {}
  Ordering(const std::vector<_T> &data, bool (*cmp)(const _T &, const _T &)) : data_(data), cmp_(cmp) {}

  std::vector<size_t> GetSortedIndices()
  {
    InitIndices();
    // Turn this object into a functor to use it for sorting, see https://stackoverflow.com/a/1902321/400948
    std::sort(indices_.begin(), indices_.end(), *this);
    return indices_;
  }

  bool operator()(size_t const &a, size_t const &b)
  {
    return cmp_(data_[a], data_[b]);
  }

private:
  Ordering();
  const std::vector<_T> &data_;
  std::vector<size_t> indices_;
  bool (*cmp_)(const _T &, const _T &);

  void InitIndices()
  {
    indices_.clear();
    indices_.reserve(data_.size());
    for (size_t i = 0; i < data_.size(); ++i)
      indices_.push_back(i);
  }
};


/** @brief Returns the indices corresponding to a sorted data vector. */
template <typename _T>
std::vector<size_t> GetSortedIndices(const std::vector<_T> &data, bool (*cmp)(const _T &, const _T &) = CmpAsc<_T>)
{
  Ordering<_T> ordering(data, cmp);
  return ordering.GetSortedIndices();
}


/** @brief Remaps the data vector by the given indices. */
template <typename _T>
std::vector<_T> ApplyIndexLookup(const std::vector<_T> &data, const std::vector<size_t> &indices)
{
  std::vector<_T> remapped;
  remapped.reserve(data.size());
  for (size_t i = 0; i < data.size(); ++i)
    remapped.push_back(data[indices[i]]);
  return remapped;
}


/** @brief Returns the data vector sorted by the given external keys. */
template <typename _To, typename _Ts>
std::vector<_To> SortByExternalVector(const std::vector<_To> &data, const std::vector<_Ts> &keys, bool (*cmp)(const _Ts &, const _Ts &) = CmpAsc<_Ts>)
{
  if (data.empty())
    return std::vector<_To>();

  if (keys.size() != data.size())
    VCP_ERROR("Vector size does not match!");

  // Sort the indices
  const std::vector<size_t> indices = GetSortedIndices<_Ts>(keys, cmp);

  // Remap the input vector
  return ApplyIndexLookup<_To>(data, indices);
}


/** @brief Returns the sorted vector (handy if you don't want to change the input data. */
template <typename _T>
std::vector<_T> SortVector(const std::vector<_T> &data, bool (*cmp)(const _T &, const _T &) = CmpAsc<_T>)
{
  // Copy, then sort
  std::vector<_T> copy;
  copy.reserve(data.size());
  for (const _T &e : data)
    copy.push_back(e);
  std::sort(copy.begin(), copy.end(), cmp);
  return copy;
}

} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_SORT_UTILS_H__
