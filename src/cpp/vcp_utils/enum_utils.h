#ifndef __VCP_UTILS_ENUM_UTILS_H__
#define __VCP_UTILS_ENUM_UTILS_H__

#include <type_traits>
#include "vcp_logging.h"

// Provide bitwise operators for type-safe enums
// Based on the excellent blog post: http://blog.bitwigglers.org/using-enum-classes-as-type-safe-bitmasks/

template<typename Enum>
Enum operator |(Enum lhs, Enum rhs)
{
  static_assert(std::is_enum<Enum>::value, "Template parameter is not an enum type");
  using underlying = typename std::underlying_type<Enum>::type;
  return static_cast<Enum> (
    static_cast<underlying>(lhs) |
    static_cast<underlying>(rhs)
  );
}

template<typename Enum>
Enum& operator |=(Enum& lhs, Enum rhs)
{
  static_assert(std::is_enum<Enum>::value, "Template parameter is not an enum type");
  using underlying = typename std::underlying_type<Enum>::type;
  lhs = static_cast<Enum> (
    static_cast<underlying>(lhs) |
    static_cast<underlying>(rhs)
  );
  return lhs;
}
//TODO add other .= overloads


template<typename Enum>
Enum operator &(Enum lhs, Enum rhs)
{
  static_assert(std::is_enum<Enum>::value, "Template parameter is not an enum type");
  using underlying = typename std::underlying_type<Enum>::type;
  return static_cast<Enum> (
    static_cast<underlying>(lhs) &
    static_cast<underlying>(rhs)
  );
}

template<typename Enum>
Enum& operator &=(Enum& lhs, Enum rhs)
{
  static_assert(std::is_enum<Enum>::value, "Template parameter is not an enum type");
  using underlying = typename std::underlying_type<Enum>::type;
  lhs = static_cast<Enum> (
    static_cast<underlying>(lhs) &
    static_cast<underlying>(rhs)
  );
  return lhs;
}

template<typename Enum>
Enum operator ^(Enum lhs, Enum rhs)
{
  static_assert(std::is_enum<Enum>::value, "Template parameter is not an enum type");
  using underlying = typename std::underlying_type<Enum>::type;
  return static_cast<Enum> (
    static_cast<underlying>(lhs) ^
    static_cast<underlying>(rhs)
  );
}

template<typename Enum>
Enum& operator ^=(Enum& lhs, Enum rhs)
{
  static_assert(std::is_enum<Enum>::value, "Template parameter is not an enum type");
  using underlying = typename std::underlying_type<Enum>::type;
  lhs = static_cast<Enum> (
    static_cast<underlying>(lhs) ^
    static_cast<underlying>(rhs)
  );
  return lhs;
}

template<typename Enum>
Enum operator ~(Enum rhs)
{
  static_assert(std::is_enum<Enum>::value, "Template parameter is not an enum type");
  using underlying = typename std::underlying_type<Enum>::type;
  return static_cast<Enum> (~static_cast<underlying>(rhs));
}


#endif // __VCP_UTILS_ENUM_UTILS_H__
