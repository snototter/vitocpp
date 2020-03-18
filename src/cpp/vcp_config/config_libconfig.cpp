#include "config_params.h"

#include <vector>
#include <iostream>
#include <exception>
#include <cstdlib>
#include <libconfig.h++>
#include <algorithm>
#include <typeinfo>
#include <sstream>
#include <utility>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/string_utils.h>
#include <vcp_utils/file_utils.h>

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::config"

namespace vcp
{
namespace config
{
// A utility to get more meaningful error messages.
template<typename T> const char *GetTypeName();
#define DEFINE_TYPE_NAME(type, name) \
    template<>const char *GetTypeName<type>(){return name;}
DEFINE_TYPE_NAME(int, "int")
DEFINE_TYPE_NAME(unsigned int, "unsigned int")
DEFINE_TYPE_NAME(long, "long")
DEFINE_TYPE_NAME(double, "double")
DEFINE_TYPE_NAME(std::string, "std::string")
DEFINE_TYPE_NAME(const char*, "const char *")
DEFINE_TYPE_NAME(bool, "bool")


// libconfig++ wrapper
namespace libconfig_wrapper
{
inline libconfig::Setting &GetSetting(const libconfig::Config &cfg, const std::string &setting_name)
{
  try
  {
    return cfg.lookup(setting_name);
  }
  catch (const libconfig::SettingNotFoundException &e)
  {
    VCP_ERROR("GetSetting(): '" << setting_name << "' not found");
  }
}

template <typename T>
T CastSetting(const libconfig::Setting &setting)
{
  try
  {
    return static_cast<T>(setting);
  }
  catch (const libconfig::SettingTypeException &e)
  {
    VCP_ERROR("Setting '" << setting.getName() << "' cannot be cast to '" << GetTypeName<T>() << "': " << e.what());
  }
}

// Specialization for double (which also allows casting from an integer - libconfig throws an exception there...)
template <>
double CastSetting(const libconfig::Setting &setting)
{
  double val;
  try
  {
    val = static_cast<double>(setting);
  }
  catch (const libconfig::SettingTypeException &e)
  {
    if (setting.isNumber())
    {
      try
      {
        const int vi = static_cast<int>(setting);
        val = static_cast<double>(vi);
      }
      catch (const libconfig::SettingTypeException &e2)
      {
        VCP_ERROR("Setting '" << setting.getName() << "' cannot be cast to 'double' or 'int': " << e2.what());
      }
    }
    else
    {
      VCP_ERROR("Setting '" << setting.getName() << "' cannot be cast to 'double': " << e.what());
    }
  }
  return val;
}

// Specialization for std::string (libconfig++ only provides const char*)
template <>
std::string CastSetting(const libconfig::Setting &setting)
{
  return std::string(CastSetting<const char*>(setting));
}


template <typename T>
T GetParam(const libconfig::Config &cfg, const std::string &param_name)
{
  const libconfig::Setting &setting = GetSetting(cfg, param_name);
  return CastSetting<T>(setting);
}

/** @brief Return a list of all parameters directly under the given parameter name. */
std::vector<std::string> ListChildren(const libconfig::Config &config, const std::string &param_name)
{
  std::vector<std::string> child_params;
  const libconfig::Setting &setting = param_name.empty() ? config.getRoot() : GetSetting(config, param_name);
  if (setting.isGroup())
  {
    for (int i = 0; i < setting.getLength(); ++i)
    {
      child_params.push_back(std::string(setting[i].getName()));
    }
  }
  return child_params;
}

/** @brief Returns a list of all parameters (full tree) under the given node. */
std::vector<std::string> ListAllParameterNames(const libconfig::Setting &node, const std::string &prefix)
{
  std::vector<std::string> names;
  if (node.isRoot() || node.isGroup())
  {
    const std::string next_prefix =
        prefix.empty() ?
          (node.isRoot() ? std::string() : std::string(node.getName()))
          : std::string(prefix + "." + std::string(node.getName()));
    for (int i = 0; i < node.getLength(); ++i)
    {
      const auto child_names = ListAllParameterNames(node[i], next_prefix);
      names.reserve(names.size() + std::distance(child_names.begin(), child_names.end()));
      names.insert(names.end(), child_names.begin(), child_names.end());
    }
  }
  else
  {
    std::string n(node.getName());
    if (prefix.empty()) // If we're at the root level
      names.push_back(n);
    else
      names.push_back(prefix + "." + n);
  }
  return names;
}


template <typename T>
std::vector<T> GetVectorFromSetting(const libconfig::Setting &setting)
{
  std::vector<T> vec;
  const int len = setting.getLength();
  for (int i = 0; i < len; ++i)
    vec.push_back(CastSetting<T>(setting[i]));
  return vec;
}

// We need a specialization for double (see CastSetting() above).
template <>
std::vector<double> GetVectorFromSetting(const libconfig::Setting &setting)
{
  std::vector<double> vec;
  const int len = setting.getLength();
  for (int i = 0; i < len; ++i)
  {
    double value;
    try
    {
      value = static_cast<double>(setting[i]);
    }
    catch (const libconfig::SettingTypeException &e)
    {
      try
      {
        int vi = static_cast<int>(setting[i]);
        value = static_cast<double>(vi);
      }
      catch (const libconfig::SettingTypeException &e2)
      {
        VCP_ERROR("Setting '" << setting.getName() << "' cannot be cast to 'double' or 'int': " << e2.what());
      }
    }
    vec.push_back(value);
  }
  return vec;
}

// Read a list of arrays, i.e. a polygon ([x1,y1],[x2,y2],...)
template <typename T>
std::vector<std::vector<T>> GetListOfVectors(const libconfig::Setting &setting)
{
  std::vector<std::vector<T>> vec;
  const int len = setting.getLength();
  for (int i = 0; i < len; ++i)
    vec.push_back(GetVectorFromSetting<T>(setting[i]));
  return vec;
}

// Read multiple polygons, i.e. a list of polygons (([],[]),([],[]),...)
template <typename T>
std::vector<std::vector<std::vector<T>>> GetMultipleListsOfVectors(const libconfig::Setting &setting)
{
  std::vector<std::vector<std::vector<T>>> vec;
  const int len = setting.getLength();
  for (int i = 0; i < len; ++i)
    vec.push_back(GetListOfVectors<T>(setting[i]));
  return vec;
}



template <typename K, typename V>
std::pair<K, V> GetPairFromList(const libconfig::Setting &setting)
{
  if (setting.getLength() != 2)
  {
    VCP_ERROR("Cannot make a pair from a setting with " << setting.getLength() << " entries");
  }
  return std::make_pair<K, V>(CastSetting<K>(setting[0]), CastSetting<V>(setting[1]));
}

// Read a list of lists (where the inner list is actually a pair), e.g. to make a dictionary (('key1', 17.0), ('key2',23.0), ...)
template <typename K, typename V>
std::vector<std::pair<K, V>> GetListOfPairs(const libconfig::Setting &setting)
{
  std::vector<std::pair<K, V>> dict;
  const int len = setting.getLength();
  for (int i = 0; i < len; ++i)
    dict.push_back(GetPairFromList<K, V>(setting[i]));
  return dict;
}


// Read a libconfig group as if it were a dictionary. Happens for example if you convert a JSON dictionary to libconfig, e.g. when
// storing RealSense sensor options:
// rgb_options = {
//   RS2_OPTION_ENABLE_AUTO_EXPOSURE = 1;
//   RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE = 0;
// };
// So our task is to iterate the group items, use their name/label as "key" and assume that the value is of the same type.
template <typename T>
std::vector<std::pair<std::string, T>> GetGroupOfPairs(const libconfig::Setting &setting)
{
  std::vector<std::pair<std::string, T>> pairs;
  const int len = setting.getLength();
  for (int i = 0; i < len; ++i)
  {
    const libconfig::Setting &node = setting[i];
    pairs.push_back(std::make_pair<std::string, T>(std::string(node.getName()), CastSetting<T>(node)));
  }
  return pairs;
}


void GetParent(const std::string &full_name, std::string &name, std::string &parent)
{
  const std::vector<std::string> tokens = vcp::utils::string::Split(full_name, '.');

  if (tokens.size() > 1)
  {
    std::stringstream ps;
    ps << tokens[0];
    for (size_t i = 1; i < tokens.size() - 1; ++i)
      ps << '.' << tokens[i];
    parent = ps.str();
    name = tokens[tokens.size()-1];
  }
  else
  {
    // We're at the root!
    parent = "";
    name = full_name;
  }
}


template <typename T>
libconfig::Setting::Type GetLibconfigType()
{
  if (typeid(T) == typeid(int))
    return libconfig::Setting::TypeInt;
  else if (typeid(T) == typeid(long))
    return libconfig::Setting::TypeInt64;
  else if (typeid(T) == typeid(double))
    return libconfig::Setting::TypeFloat;
  else if (typeid(T) == typeid(std::string))
    return libconfig::Setting::TypeString;
  else if (typeid(T) == typeid(bool))
    return libconfig::Setting::TypeBoolean;

  return libconfig::Setting::TypeNone;
}


void CreateSettingsGroup(libconfig::Config &config, const std::string &group)
{
  const auto tokens = vcp::utils::string::Split(group, '.');
  std::stringstream group_names, parent_names;
  for (size_t i = 0; i < tokens.size(); ++i)
  {
    if (i > 0)
      group_names << '.';
    group_names << tokens[i];

    if (!config.exists(group_names.str()))
    {
      const std::string parent = parent_names.str();

      VCP_LOG_DEBUG("CreateSettingsGroup(): Creating group '" << tokens[i] << "' under parent '" << parent_names.str() << "'");
      libconfig::Setting &parent_node = parent.empty() ? config.getRoot() : config.lookup(parent);
      parent_node.add(tokens[i], libconfig::Setting::TypeGroup);
    }

    if (i > 0)
      parent_names << '.';
    parent_names << tokens[i];
  }
}

template <typename T>
void SetBuiltInType(libconfig::Config &config, const std::string &name, const T &value)
{
  if (!config.exists(name))
  {
    VCP_LOG_DEBUG("SetBuiltInType(): Creating new setting '" << name << "' of type " << GetTypeName<T>() << ", value=" << value);
    std::string parent, sname;
    GetParent(name, sname, parent);

    if (!config.exists(parent))
      CreateSettingsGroup(config, parent);

    libconfig::Setting &pnode = parent.empty() ? config.getRoot() : config.lookup(parent);
    pnode.add(sname, GetLibconfigType<T>());
  }
  libconfig::Setting &setting = config.lookup(name);
  setting = value;
}


void RemoveSetting(libconfig::Config &config, const std::string &name)
{
  if (config.exists(name))
  {
    libconfig::Setting &setting = config.lookup(name);
    libconfig::Setting &p = setting.getParent();
    p.remove(name);
  }
}


template <typename T>
void FillArray(libconfig::Setting &array, const std::vector<T> &values)
{
  for (size_t i = 0; i < values.size(); ++i)
  {
    libconfig::Setting &val = array.add(GetLibconfigType<T>());
    val = values[i];
  }
}

template <typename T>
void SetArray(libconfig::Config &config, const std::string &name, const std::vector<T> &values)
{
  std::string parent, sname;
  GetParent(name, sname, parent);

  // We must replace all entries of an existing array.
  RemoveSetting(config, name);

  if (!config.exists(parent))
    CreateSettingsGroup(config, parent);

  // Create the array under the parent.
  libconfig::Setting &pnode = parent.empty() ? config.getRoot() : config.lookup(parent);
  libconfig::Setting &arr = pnode.add(sname, libconfig::Setting::TypeArray);

  // Fill it with values
  FillArray(arr, values);
}

template <typename T>
void SetListOfVectors(libconfig::Config &config, const std::string &name, const std::vector<std::vector<T>> &values)
{
  std::string parent, sname;
  GetParent(name, sname, parent);

  // We must replace all entries of an existing array.
  RemoveSetting(config, name);

  if (!config.exists(parent))
    CreateSettingsGroup(config, parent);

  // Create the list under the parent.
  libconfig::Setting &pnode = parent.empty() ? config.getRoot() : config.lookup(parent);
  libconfig::Setting &list = pnode.add(sname, libconfig::Setting::TypeList);

  // Fill it with arrays
  for (size_t i = 0; i < values.size(); ++i)
  {
    libconfig::Setting &arr = list.add(libconfig::Setting::TypeArray);
    FillArray(arr, values[i]);
  }
}

template <typename T>
void SetMultipleListsOfVectors(libconfig::Config &config, const std::string &name, const std::vector<std::vector<std::vector<T>>> &values)
{
  std::string parent, sname;
  GetParent(name, sname, parent);

  // We must replace all entries of an existing array.
  RemoveSetting(config, name);

  if (!config.exists(parent))
    CreateSettingsGroup(config, parent);

  // Create the list under the parent.
  libconfig::Setting &pnode = parent.empty() ? config.getRoot() : config.lookup(parent);
  libconfig::Setting &list = pnode.add(sname, libconfig::Setting::TypeList);

  // Fill it with lists
  for (size_t i = 0; i < values.size(); ++i)
  {
    libconfig::Setting &sublist = list.add(libconfig::Setting::TypeList);
    const auto &subval = values[i];

    // Fill each sublist with the respective arrays
    for (size_t j = 0; j < subval.size(); ++j)
    {
      libconfig::Setting &arr = sublist.add(libconfig::Setting::TypeArray);
      FillArray(arr, subval[j]);
    }
  }
}


bool IsArray(libconfig::Setting &group)
{
  // An array has elements of the same type
  if (group.getLength() == 0)
    return true;

  if (std::char_traits<char>::length(group[0].getName()) > 0)
    return false; // Named nodes can only occur in a list()

  libconfig::Setting::Type t = group[0].getType();
  for (int i = 1; i < group.getLength(); ++i)
  {
    if (std::char_traits<char>::length(group[i].getName()) > 0)
      return false; // Named nodes can only occur in a list()
    if (group[i].getType() != t)
      return false;
  }
  return true;
}


template <typename T1, typename T2>
void SetListOfPairs(libconfig::Config &config, const std::string &name, const std::vector<std::pair<T1, T2>> &values)
{
  std::string parent, sname;
  GetParent(name, sname, parent);

  // We must replace all entries of an existing array.
  RemoveSetting(config, name);

  if (!config.exists(parent))
    CreateSettingsGroup(config, parent);

  // Create the list under the parent.
  libconfig::Setting &pnode = parent.empty() ? config.getRoot() : config.lookup(parent);
  libconfig::Setting &list = pnode.add(sname, libconfig::Setting::TypeList);

  // Fill it with the pairs
  for (size_t i = 0; i < values.size(); ++i)
  {
    libconfig::Setting &nested_list = list.add(libconfig::Setting::TypeList);
    libconfig::Setting &val1 = nested_list.add(GetLibconfigType<T1>());
    val1 = values[i].first;
    libconfig::Setting &val2 = nested_list.add(GetLibconfigType<T2>());
    val2 = values[i].second;
  }
}


// Scalar value to string conversion
std::string ValueToString(const libconfig::Setting &setting)
{
  std::stringstream str;
  if (setting.getType() == GetLibconfigType<int>())
    str << CastSetting<int>(setting);
  else if (setting.getType() == GetLibconfigType<long>())
    str << CastSetting<long>(setting);
  else if (setting.getType() == GetLibconfigType<double>())
    str << CastSetting<double>(setting);
  else if (setting.getType() == GetLibconfigType<std::string>())
    str << CastSetting<const char*>(setting);
  else if (setting.getType() == GetLibconfigType<bool>())
  {
    if (CastSetting<bool>(setting))
      str << "true";
    else
      str << "false";
  }
  else if (setting.isAggregate())
  {
    if (setting.isGroup())
      str << "{";
    else if (setting.isArray())
      str << "[";
    else
      str << "(";

    for (int i = 0; i < setting.getLength(); ++i)
    {
      str << ValueToString(setting[i]);
      if (i < setting.getLength() - 1)
        str << ", ";
    }
    if (setting.isGroup())
      str << "}";
    else if (setting.isArray())
      str << "]";
    else
      str << ")";
  }
  else
    str << "Unknown setting type!";
  return str.str();
}

// DFS to turn the config into a human readable representation
void ConfigToString(libconfig::Setting &node, std::ostream &out, size_t indent)
{
  for (size_t i = 0; i < indent; ++i)
    out << "  ";

  if (!node.isRoot())
    out << node.getName() << " = ";

  const bool is_array = (node.isGroup() && IsArray(node)) || node.isArray();

  const bool is_list = node.isList();

  const bool is_collection = node.isGroup() && !is_array;

  const bool is_num_or_string = node.isScalar() || node.isNumber(); // also captures TypeBool

  if (is_collection)
  {
    out << std::endl;
    for (size_t i = 0; i < indent; ++i)
      out << "  ";
    if (is_collection)
      out << "{" << std::endl;
    else
      out << "(" << std::endl;

    for (int i = 0; i < node.getLength(); ++i)
      ConfigToString(node[i], out, indent+1);

    for (size_t i = 0; i < indent; ++i)
      out << "  ";
    if (is_collection)
      out << "};" << std::endl;
    else
      out << ")" << std::endl;
  }
  else if (is_list)
  {
    out << "(";
    for (int i = 0; i < node.getLength(); ++i)
    {
      out << ValueToString(node[i]);
      if (i < node.getLength() - 1)
        out << ", ";
    }
    out << ");" << std::endl;
  }
  else if (is_array)
  {
    out << "[";
    for (int i = 0; i < node.getLength(); ++i)
    {
      out << ValueToString(node[i]);
      if (i < node.getLength() - 1)
        out << ", ";
    }
    out << "];" << std::endl;
  }
  else if (is_num_or_string)
  {
    out << ValueToString(node) << ";" << std::endl;
  }
  else
  {
    VCP_ERROR("ConfigToString(): Type of config::Setting is not supported, check param '" + node.getPath() + "'");
  }
}


void SingleSettingToString(libconfig::Setting &node, std::ostream &out)
{
  const bool is_array = (node.isGroup() && IsArray(node)) || node.isArray();
  const bool is_list = node.isList();

  const bool is_collection = node.isGroup() && !is_array;

  const bool is_num_or_string = node.isScalar() || node.isNumber();

  if (is_collection)
  {
    out << "[";

    for (int i = 0; i < node.getLength(); ++i)
      SingleSettingToString(node[i], out);

    out << "]";
  }
  else if (is_array)
  {
    out << "[";
    for (int i = 0; i < node.getLength(); ++i)
    {
      out << ValueToString(node[i]);
      if (i < node.getLength() - 1)
        out << ", ";
    }
    out << "]";
  }
  else if (is_list)
  {
    out << "(";
    for (int i = 0; i < node.getLength(); ++i)
    {
      out << ValueToString(node[i]);
      if (i < node.getLength() - 1)
        out << ", ";
    }
    out << ")";
  }
  else if (is_num_or_string)
  {
    out << ValueToString(node);
  }
  else
  {
    VCP_ERROR("Type of config::Setting is not supported in vcp::config::libconfig_wrapper::SingleSettingToString(), check param '" + node.getPath() + "'");
  }
}


class ConfigParamsPP : public ConfigParams
{
public:
  explicit ConfigParamsPP(const std::string& config_file) : ConfigParams(), filename_(config_file)
  {
    try
    {
      config_.readFile(filename_.c_str());
    }
    catch (const libconfig::ParseException& e)
    {
      VCP_ERROR("Error reading configuration file (" << filename_ << ")!" << std::endl <<
        "ParseException at Line " << e.getLine() << ": " << e.getError());
    }
    catch (const libconfig::SettingException& e)
    {
      VCP_ERROR("Error reading configuration file (" << filename_ << ")!" << std::endl <<
        "SettingException at " << e.getPath());
    }
    catch (const std::exception& e)
    {
      VCP_ERROR("Error reading configuration file (" << filename_ << ")!" << std::endl <<
        e.what());
    }
    catch (...)
    {
      VCP_ERROR("Error reading configuration file (" << filename_ << ")!" << std::endl <<
        "Unknown Exception");
    }
  }

  explicit ConfigParamsPP() : ConfigParams() {}

  virtual ~ConfigParamsPP() {}

  int GetInteger(const std::string &param_name) const override { return GetParam<int>(config_, param_name); }
  unsigned int GetUnsignedInteger(const std::string &param_name) const override { return GetParam<unsigned int>(config_, param_name); }
  double GetDouble(const std::string &param_name) const override { return GetParam<double>(config_, param_name); }
  bool GetBoolean(const std::string &param_name) const override { return GetParam<bool>(config_, param_name); }
  std::string GetString(const std::string &param_name) const override { return std::string(GetParam<const char*>(config_, param_name)); }

  std::vector<int> GetIntegerArray(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    return GetVectorFromSetting<int>(setting);
  }

  std::vector<double> GetDoubleArray(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    return GetVectorFromSetting<double>(setting);
  }

  std::vector<std::string> GetStringArray(const std::string &param_name) const override
  {
    const libconfig::Setting &settings = GetSetting(config_, param_name);
    const int len = settings.getLength();

    std::vector<std::string> ret(len);
    for(int i = 0; i < len; ++i)
      ret[i] = std::string(CastSetting<const char*>(settings[i]));
    return ret;
  }

  std::vector<int> GetSize2D(const std::string &param_name) const override
  {
    auto ret = GetIntegerArray(param_name);
    if (ret.size() != 2)
    {
      VCP_ERROR("GetSize2D(): Parameter '" << param_name << "' is not a 2-element vector!");
    }
    return ret;
  }

  std::vector<int> GetRectangle(const std::string &param_name) const override
  {
    auto ret = GetIntegerArray(param_name);
    if (ret.size() != 4)
    {
      VCP_ERROR("GetRectangle(): Parameter '" << param_name << "' is not a 4-element vector!");
    }
    return ret;
  }

  std::vector<std::vector<int>> GetIntegerPolygon(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    return GetListOfVectors<int>(setting);
  }

  std::vector<std::vector<std::vector<int>>> GetIntegerPolygons(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    return GetMultipleListsOfVectors<int>(setting);
  }

  std::vector<std::pair<std::string, double>> GetDoubleKeyValueList(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    switch(setting.getType())
    {
      case libconfig::Setting::Type::TypeList:
        return GetListOfPairs<std::string, double>(setting);

    case libconfig::Setting::Type::TypeGroup:
      return GetGroupOfPairs<double>(setting);

    default:
      VCP_ERROR("GetDoubleKeyValueList(): Don't know how to convert '" << param_name << "' to a dictionary-like data structure.");
    }
  }

  std::vector<std::pair<std::string, int>> GetIntegerKeyValueList(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    switch(setting.getType())
    {
    case libconfig::Setting::Type::TypeList:
      return GetListOfPairs<std::string, int>(setting);

    case libconfig::Setting::Type::TypeGroup:
      return GetGroupOfPairs<int>(setting);

    default:
      VCP_ERROR("GetIntegerKeyValueList(): Don't know how to convert '" << param_name << "' to a dictionary-like data structure.");
    }
  }

  std::vector<std::vector<std::string>> GetNestedStringList(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    return GetListOfVectors<std::string>(setting);
  }

  std::vector<std::vector<int>> GetNestedIntegerList(const std::string &param_name) const override
  {
    const libconfig::Setting &setting = GetSetting(config_, param_name);
    return GetListOfVectors<int>(setting);
  }

  bool SettingExists(const std::string &param_name) const override
  {
    return config_.exists(param_name);
  }

  int SettingLength(const std::string &param_name) const override
  {
    return config_.lookup(param_name).getLength();
  }
//TODO all vcp_error/warnings should be prefixed by function name

  bool SaveConfiguration(const std::string &filename) override
  {
    try
    {
      config_.writeFile(filename.c_str());
      return true;
    }
    catch (const libconfig::FileIOException &e)
    {
      VCP_LOG_FAILURE("SaveConfiguration(): FileIOException while writing configuration file (" << filename << ")!");
    }
    return false;
  }

  void SetInteger(const std::string &param_name, int value) override   { SetBuiltInType<int>(config_, param_name, value); }
  void SetDouble(const std::string &param_name, double value) override { SetBuiltInType<double>(config_, param_name, value); }
  void SetBoolean(const std::string &param_name, bool value) override { SetBuiltInType<bool>(config_, param_name, value); }
  void SetString(const std::string &param_name, const std::string &value) override { SetBuiltInType<std::string>(config_, param_name, value); }

  void SetIntegerArray(const std::string &param_name, const std::vector<int> &values) override { SetArray<int>(config_, param_name, values); }
  void SetDoubleArray(const std::string &param_name, const std::vector<double> &values) override { SetArray<double>(config_, param_name, values); }
  void SetStringArray(const std::string &param_name, const std::vector<std::string> &values) override { SetArray<std::string>(config_, param_name, values); }

  void SetSize2D(const std::string &param_name, int width, int height) override { SetIntegerArray(param_name, {width, height}); }
  void SetRectangle(const std::string &param_name, int left, int top, int width, int height) override { SetIntegerArray(param_name, {left, top, width, height}); }

  void SetIntegerPolygon(const std::string &param_name, const std::vector<std::vector<int>> &values) override { SetListOfVectors<int>(config_, param_name, values); }
  void SetIntegerPolygons(const std::string &param_name, const std::vector<std::vector<std::vector<int>>> &values) override { SetMultipleListsOfVectors<int>(config_, param_name, values); }

  void SetIntegerKeyValueList(const std::string &param_name, const std::vector<std::pair<std::string, int>> &values) override { SetListOfPairs<std::string, int>(config_, param_name, values); }
  void SetDoubleKeyValueList(const std::string &param_name, const std::vector<std::pair<std::string, double>> &values) override { SetListOfPairs<std::string, double>(config_, param_name, values); }

  std::string ToString() const override
  {
    std::stringstream str;
    ConfigToString(config_.getRoot(), str, 0);
    return str.str();
  }

  std::string AsString(const std::string &param_name) const override
  {
    std::stringstream str;
    try
    {
      libconfig::Setting &setting = config_.lookup(param_name);
      SingleSettingToString(setting, str);
    }
    catch (const libconfig::SettingNotFoundException &)
    {
      VCP_ERROR("Setting '" + param_name + "' not found");
    }
    return str.str();
  }

  std::vector<std::string> ListConfigGroupParameters(const std::string &param_name) const override
  {
    return ListChildren(config_, param_name);
  }


  std::vector<std::string> ListConfigParameters() const override
  {
    return ListAllParameterNames(config_.getRoot(), std::string());
  }


  void EnsureAbsolutePaths(const std::vector<std::string> &param_names,
                           const std::string &absolute_base_path,
                           bool verbose) override
  {
    // Get a list of all parameter names.
    const std::vector<std::string> configured_params = ListConfigParameters();
    // We'll split the parameter names for matching (remove optional parameter "groups", i.e.
    // for "cam1.param" we would match the given "param_names" against "param".
    std::vector<std::string> configured_params_match_tokens;
    for (const auto &cp : configured_params)
    {
      const auto tokens = vcp::utils::string::Split(cp, '.');
      if (!tokens.empty())
      {
        configured_params_match_tokens.push_back(
              vcp::utils::string::Lower(tokens[tokens.size()-1]));
      }
    }

    // Iterate all given parameter names which should be checked:
    for (const auto &pn : param_names)
    {
      // We'll match lower case
      const std::string pn_lower = vcp::utils::string::Lower(pn);

      for (size_t i = 0; i < configured_params_match_tokens.size(); ++i)
      {
        if (configured_params_match_tokens[i].compare(pn_lower) != 0)
          continue;

        const std::string k = configured_params[i];
        const std::string path_orig = GetString(k);
        const bool is_url = vcp::utils::string::StartsWith(path_orig, "file://");
        const std::string path = is_url ? path_orig.substr(7) : path_orig;

        if (vcp::utils::file::IsAbsolute(path))
          continue;

        const std::string abs_path = vcp::utils::file::RealPath(
              vcp::utils::file::FullFile(absolute_base_path, path));
        SetString(k, is_url ? "file://" + abs_path : abs_path);

        if (verbose)
          VCP_LOG_INFO("Updated config path '" << k << "' to '" << GetString(k) << "'");
      }
    }
  }


private:
  libconfig::Config config_;
  std::string filename_;
};
} // namespace libconfig_wrapper

std::unique_ptr<ConfigParams> LoadConfigParamsCpp(const std::string &config_filename)
{
  return std::unique_ptr<libconfig_wrapper::ConfigParamsPP>(new libconfig_wrapper::ConfigParamsPP(config_filename));
}

std::unique_ptr<ConfigParams> CreateEmptyConfigParamsCpp()
{
  return std::unique_ptr<libconfig_wrapper::ConfigParamsPP>(new libconfig_wrapper::ConfigParamsPP());
}

} // namespace config
} // namespace vcp
