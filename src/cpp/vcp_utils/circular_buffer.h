#ifndef __VCP_UTILS_CIRCULAR_BUFFER_H__
#define __VCP_UTILS_CIRCULAR_BUFFER_H__

#include <iterator>
#include <memory>
#include <vector>
#include <stdexcept>
#include <stddef.h>
#include <iostream>

namespace vcp
{
namespace utils
{

/** @brief Templated STL compatible iterator for the circular_buffer. */
template <typename T, typename T_nonconst, typename elem_type = typename T::value_type>
class circular_buffer_iterator {
public:
  // STL requested typedefs
  typedef circular_buffer_iterator<T,T_nonconst,elem_type> self_type;
  typedef T cbuf_type;
  typedef std::random_access_iterator_tag iterator_category;
  typedef typename cbuf_type::value_type value_type; // Element type the iterator "points to".
  typedef typename cbuf_type::size_type size_type;   // Container index type
  typedef typename cbuf_type::pointer pointer;
  typedef typename cbuf_type::const_pointer const_pointer;
  typedef typename cbuf_type::reference reference;
  typedef typename cbuf_type::const_reference const_reference;
  typedef typename cbuf_type::difference_type difference_type;

  /** @brief Constructor. */
  circular_buffer_iterator(cbuf_type *buf, size_type pos) : buf_(buf), pos_(pos) {}

  // Use auto-generated copy constructor, copy/assignment operator and destructor.


  /** @brief Provides an explicit cast from iterator to const_iterator to enable "const_iterator = buffer.begin()"; */
  circular_buffer_iterator(const circular_buffer_iterator<T_nonconst, T_nonconst, typename T_nonconst::value_type> &other)
    : buf_(other.buf_), pos_(other.pos_) {}
  friend class circular_buffer_iterator<const T, T, const elem_type>;

  /** @brief Element access. */
  elem_type &operator*() { return (*buf_)[pos_]; }

  /** @brief Element access. */
  elem_type *operator->() { return &(operator*()); }


  //--------------------------------------------------------------------------------
  // Increment/Decrement
  /** @brief Prefix ++it operator. */
  self_type &operator++()
  {
    pos_++;
    return *this;
  }

  /** @brief Postfix it++ operator. */
  self_type operator++(int)
  {
    self_type tmp(*this);
    ++(*this);
    return tmp;
  }

  /** @brief Prefix --it operator. */
  self_type &operator--()
  {
    pos_--;
    return *this;
  }

  /** @brief Prefix it-- operator. */
  self_type operator--(int)
  {
    self_type tmp(*this);
    --(*this);
    return tmp;
  }

  /** @brief Overloaded + operator. */
  self_type operator+(difference_type n) const
  {
    self_type tmp(*this);
    tmp.pos_ += n;
    return tmp;
  }

  /** @brief Overloaded += operator. */
  self_type &operator+=(difference_type n)
  {
    pos_ += n;
    return *this;
  }

  /** @brief Overloaded - operator. */
  self_type operator-(difference_type n) const
  {
    self_type tmp(*this);
    tmp.pos_ -= n;
    return tmp;
  }

  /** @brief Overlaoded -= operator. */
  self_type &operator-=(difference_type n)
  {
    pos_ -= n;
    return *this;
  }

  /** @brief Overloaded - operator. */
  difference_type operator-(const self_type &c) const
  {
    return pos_ - c.pos_;
  }

  //--------------------------------------------------------------------------------
  // Comparisons
  bool operator==(const self_type &other) const
  {
    return pos_ == other.pos_ && buf_ == other.buf_;
  }
  bool operator!=(const self_type &other) const
  {
    return pos_ != other.pos_ && buf_ == other.buf_;
  }
  bool operator>(const self_type &other) const
  {
    return pos_ > other.pos_;
  }
  bool operator>=(const self_type &other) const
  {
    return pos_ >= other.pos_;
  }
  bool operator<(const self_type &other) const
  {
    return pos_ < other.pos_;
  }
  bool operator<=(const self_type &other) const
  {
    return pos_ <= other.pos_;
  }

private:
  cbuf_type *buf_;
  size_type pos_;
};

template <typename circular_buffer_iterator_t>
circular_buffer_iterator_t operator+(const typename circular_buffer_iterator_t::difference_type &a, const circular_buffer_iterator_t &b)
{
  return circular_buffer_iterator_t(a) + b;
}

template <typename circular_buffer_iterator_t>
circular_buffer_iterator_t operator-(const typename circular_buffer_iterator_t::difference_type &a, const circular_buffer_iterator_t &b)
{
  return circular_buffer_iterator_t(a) - b;
}


//--------------------------------------------------------------------------------
// STL-like circular buffer
template <typename T, int DefaultCapacity = 100, typename Alloc = std::allocator<T> >
class circular_buffer {
public:
  //---------------------------------------------------------------------------
  // STL requested typedefs.
  typedef circular_buffer<T, DefaultCapacity, Alloc> self_type;
  typedef Alloc allocator_type;
  typedef typename Alloc::value_type value_type;
  typedef typename Alloc::pointer pointer;
  typedef typename Alloc::const_pointer const_pointer;
  typedef typename Alloc::reference reference;
  typedef typename Alloc::const_reference const_reference;
  typedef typename Alloc::size_type size_type;
  typedef typename Alloc::difference_type difference_type;
  typedef circular_buffer_iterator<self_type, self_type> iterator;
  typedef circular_buffer_iterator<const self_type, self_type, const value_type> const_iterator;
  typedef std::reverse_iterator<iterator> reverse_iterator;
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator;


  //---------------------------------------------------------------------------
  // Construction/Destruction/Assignment
  explicit circular_buffer(size_type capacity = DefaultCapacity)
    : data_(alloc_.allocate(capacity)), capacity_(capacity), head_(0), tail_(0), size_(0)
  {}

  circular_buffer(const circular_buffer &other)
    : data_(alloc_.allocate(other.capacity_)), capacity_(other.capacity_),
      head_(other.head_), tail_(other.tail_), size_(other.size_)
  {
    try
    {
      assign_into(other.begin(), other.end());
    }
    catch(...)
    {
      destroy_all_elements();
      alloc_.deallocate(data_, capacity_);
      throw;
    }
  }

  template <class InputIterator>
  circular_buffer(InputIterator from, InputIterator to)
    : data_(alloc_.allocate(1)), capacity_(1),
      head_(0), tail_(0), size_(0)
  {
    circular_buffer tmp;
    tmp.assign_into_reserving(from, to);
    swap(tmp);
  }

  ~circular_buffer()
  {
    destroy_all_elements();
    alloc_.deallocate(data_, capacity_);
  }

  circular_buffer &operator=(const self_type &other)
  {
    circular_buffer tmp(other);
    swap(tmp);
    return *this;
  }
  void swap(circular_buffer &other)
  {
    std::swap(data_, other.data_);
    std::swap(capacity_, other.capacity_);
    std::swap(head_, other.head_);
    std::swap(tail_, other.tail_);
    std::swap(size_, other.size_);
  }

  allocator_type get_allocator() const { return alloc_; }

  //---------------------------------------------------------------------------
  // Iterators
  iterator begin() { return iterator(this, 0); }
  iterator end()   { return iterator(this, size()); }

  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator cbegin() const { return begin(); }

  const_iterator end() const   { return const_iterator(this, size()); }
  const_iterator cend() const { return end(); }


  reverse_iterator rbegin() { return reverse_iterator(end()); }
  reverse_iterator rend()   { return reverse_iterator(begin()); }

  const_reverse_iterator rbegin() const  { return const_reverse_iterator(end()); }
  const_reverse_iterator crbegin() const { return rbegin(); }

  const_reverse_iterator rend() const  { return const_reverse_iterator(begin()); }
  const_reverse_iterator crend() const { return rend(); }


  //---------------------------------------------------------------------------
  // Buffer size.
  size_type size() const { return size_; }
  size_type capacity() const { return capacity_; }
  bool empty() const { return !size_; }
  size_type max_size() const { return alloc_.max_size(); }

  void reserve(size_type new_size)
  {
    if (capacity() < new_size)
    {
      circular_buffer tmp(new_size);
      tmp.assign_into(begin(), end());
      swap(tmp);
    }
  }

  //---------------------------------------------------------------------------
  // Element access
  reference front() { return data_[tail_]; }
  reference back()  { return data_[head_]; }
  const_reference front() const { return data_[tail_]; }
  const_reference back() const  { return data_[head_]; }

  void push_back(const value_type &item)
  {
    // If empty, start at the first (0-based) index to insert!
    head_ = !size_ ? 0 : next_position(head_);

    if (size_ == capacity_)
    {
      data_[head_] = item;
      tail_ = next_position(tail_);
    }
    else
    {
      alloc_.construct(data_ + head_, item);
      ++size_;
    }
  }

  void pop_front()
  {
    size_type destroy_pos = tail_;
    --size_;
    if (size_ == 0)
      tail_ = 0;
    else
      tail_ = next_position(tail_);
    alloc_.destroy(data_ + destroy_pos);
  }

  void pop_back()
  {
    size_type destroy_pos = head_;
    --size_;
    if (head_ == 0)
      head_ = capacity_ - 1;
    else
      --head_;
    if (size_ == 0)
      tail_ = 0;
    alloc_.destroy(data_ + destroy_pos);
  }

  void clear()
  {
    for (size_type n = 0; n < size_; ++n)
      alloc_.destroy(data_ + index_to_subscript(n));
    head_ = tail_ = size_ = 0;
  }

  reference operator[](size_type n) { return at_unchecked(n); }
  const_reference operator[](size_type n) const { return at_unchecked(n); }

  reference at(size_type n) { return at_checked(n); }
  const_reference at(size_type n) const { return at_checked(n); }

private:
  allocator_type  alloc_;
  value_type *data_;
  size_type capacity_;
  size_type head_;
  size_type tail_;
  size_type size_;

  reference at_unchecked(size_type index) const { return data_[index_to_subscript(index)]; }

  reference at_checked(size_type index) const
  {
    if (size >= size_)
    {
      throw std::out_of_range();
    }
    return at_unchecked(index);
  }

  // Bound index into data capacity.
  inline size_type normalise(size_type n) const { return n % capacity_; }

  // Converts external index to an array subscript
  size_type index_to_subscript(size_type index) const { return normalise(index + tail_); }

  size_type next_position(size_type index)
  {
    return (index+1 == capacity_) ? 0 : index + 1;
  }

  template <typename iter>
  void assign_into(iter from, iter to)
  {
    if (size_)
      clear();
    while (from != to)
    {
      push_back(*from);
      ++from;
    }
  }

  template <typename iter>
  void assign_into_reserving(iter from, iter to)
  {
    if (size_)
      clear();
    while (from != to)
    {
      if (size_ == capacity_)
        reserve(static_cast<size_type>(capacity_ * 1.5));
      push_back(*from);
      ++from;
    }
  }

  void destroy_all_elements()
  {
    for (size_type n = 0; n < size_; ++n)
      alloc_.destroy(data_ + index_to_subscript(n));
  }
};
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_CIRCULAR_BUFFER_H__
