#include "config_params.h"

#include <vector>
#include <iostream>
#include <exception>
#include <fstream>
#include <cmath>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace vcp
{
namespace config
{
namespace json_wrapper
{
bool IsInteger(double value)
{
  double intpart;
  double fractpart = std::modf(value, &intpart);
  return std::fabs(fractpart) < 1e-10;
}

bool IsIntegerArray(const json &arr)
{
  // Empty array
  if (arr.begin() == arr.end())
    return true;

  // All elements must be integer
  for (const auto &a : arr)
  {
    if (!a.is_number() || !IsInteger(a))
      return false;
  }
  return true;
}

bool IsDoubleArray(const json &arr)
{
  // Empty array
  if (arr.begin() == arr.end())
    return true;

  // All elements must be numeric
  for (const auto &a : arr)
  {
    if (!a.is_number())
      return false;
  }
  return true;
}


bool IsStringArray(const json &arr)
{
  // Empty array
  if (arr.begin() == arr.end())
    return true;

  // All elements must be strings
  for (const auto &a : arr)
  {
    if (!a.is_string())
      return false;
  }
  return true;
}


bool IsKeyValueList(const json &kvl)
{
  if (kvl.begin() == kvl.end())
    return true;

  // All elements must be array/tuple-like with 2 elements, [0] is string, [1] is value
  for (const auto &e : kvl)
  {
    if (!e.is_array() || e.size() != 2)
      return false;
  }
  return true;
}


template<bool (*ValidKey)(const json &), bool (*ValidValue)(const json &)>
bool IsTypedKeyValueList(const json &kvl)
{
  if (kvl.begin() == kvl.end())
    return true;

  // All elements must be array/tuple-like with 2 elements, [0] is string, [1] is value
  for (const auto &e : kvl)
  {
    if (!e.is_array() || e.size() != 2)
      return false;
    if (!ValidKey(e[0]))
      return false;
    if (!ValidValue(e[1]))
      return false;
  }
  return true;
}

// Needed for above template magic
bool IsInt(const json &v)
{
  return v.is_number() && v.is_number_integer();
}

bool IsDouble(const json &v)
{
  return v.is_number();
}

bool IsString(const json &v)
{
  return v.is_string();
}


void ConstructConfigParam(const json &elem, ConfigParams &params, const std::string &root)
{
  std::string prefix = root.empty() ? "" : root + ".";

  for (json::const_iterator it = elem.begin(); it != elem.end(); ++it)
  {
    const std::string param_name = prefix + it.key();
    const auto &v = it.value();
    if (v.is_boolean())
    {
      params.SetBoolean(param_name, v.get<bool>());
    }
    else if (v.is_number())
    {
      if (IsInteger(v.get<double>()))
        params.SetInteger(param_name, v.get<int>());
      else
        params.SetDouble(param_name, v.get<double>());
    }
    else if (v.is_array())
    {
      if (IsIntegerArray(v))
      {
        std::vector<int> values;
        for (auto& element : v)
          values.push_back(element);
        params.SetIntegerArray(param_name, values);
      }
      else if (IsDoubleArray(v))
      {
        std::vector<double> values;
        for (auto& element : v)
          values.push_back(element);
        params.SetDoubleArray(param_name, values);
      }
      else if (IsStringArray(v))
      {
        std::vector<std::string> values;
        for (auto& element : v)
          values.push_back(element);
        params.SetStringArray(param_name, values);
      }
      else if (IsTypedKeyValueList<IsString, IsInt>(v))
      {
        std::vector<std::pair<std::string, int>> values;
        for (auto& element : v)
          values.push_back(std::make_pair<std::string, int>(element[0], element[1]));
        params.SetIntegerKeyValueList(param_name, values);
      }
      else if (IsTypedKeyValueList<IsString, IsDouble>(v))
      {
        std::vector<std::pair<std::string, double>> values;
        for (auto& element : v)
          values.push_back(std::make_pair<std::string, double>(element[0], element[1]));
        params.SetDoubleKeyValueList(param_name, values);
      }
      else
        throw std::runtime_error("Only int/double/string arrays are currently supported");
    }
    else if (v.is_string())
    {
      params.SetString(param_name, v.get<std::string>());
    }
    else if (v.is_object())
    {
      ConstructConfigParam(v, params, prefix + it.key());
    }
    else
    {
      throw std::runtime_error("Type of " + root + "." + it.key() + " is not supported");
    }
  }
}

std::unique_ptr<ConfigParams> JsonToConfigParamsCpp(const json &js)
{
  std::unique_ptr<ConfigParams> cfg = CreateEmptyConfigParamsCpp();
  ConstructConfigParam(js, *cfg, std::string());

  return cfg; // explicit way would be to call return move(cfg); but the returned object is automatically treated as rvalue (hence moved instead of copied), since the function terminates.
}
} // namespace json_wrapper

std::unique_ptr<ConfigParams> LoadConfigParamsJson(const std::string &input_json, bool is_json_string)
{
  json js;
  if (is_json_string)
  {
    js = json::parse(input_json);
  }
  else
  {
    std::ifstream ifs(input_json);
    js = json::parse(ifs);
  }

  return json_wrapper::JsonToConfigParamsCpp(js);
}

} // namespace config
} // namespace vcp
