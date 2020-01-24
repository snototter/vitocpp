#include <iostream>
#include <string>
#include <vector>

#include <vcp_config/config_params.h>
#include <vcp_utils/vcp_logging.h>

#include <libconfig.h++>

int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);
  std::unique_ptr<vcp::config::ConfigParams> ec = vcp::config::CreateEmptyConfigParamsCpp();
  ec->SetInteger("int_at_root", 1);
  ec->SetInteger("group_at_root.int", 2);
  ec->SetString("group_at_root.child.a_string", "somestring");
  ec->SetString("group_at_root.child.a_string", "overwritten!");
  ec->SetIntegerArray("int_arr_at_root", {1,2,3,4});
  ec->SetIntegerArray("group_at_root.child.int_arr", {5,6,7});
//  ec->SetDoubleArray("group1.s1.s23.darr", {1, 2.3});
//  ec->SetStringArray("group1.strarr", {"foo", "bar"});
  ec->SetRectangle("rect", 0,1,100,3425);
  ec->SetSize2D("group_at_root.size", 50,70);
  ec->SetIntegerPolygon("poly", {{0,1}, {0,2}, {17,59}});
  ec->SetDouble("SoMe.Really.Very.deep.val", 23);

  ec->SetIntegerPolygons("multipoly", {{{0,1}, {0,2}}, {{1,2,3},{4,5,6},{7,8}}});

  ec->SaveConfiguration("fubar.txt");
  for (const auto &s : ec->ListConfigParameters())
    VCP_LOG_INFO_DEFAULT(s);


//  // Libconfig supports:
//  // * A scalar value: integer, 64-bit integer, floating-point number, boolean, or string
//  // * An array, which is a sequence of scalar values, all of which must have the same type
//  // * A group, which is a collection of settings
//  // * A list, which is a sequence of values of any type, including other lists
//  libconfig::Config cfg;
//  libconfig::Setting &root = cfg.getRoot();
//  libconfig::Setting &vala = root.add("SomeInt", libconfig::Setting::TypeInt);
//  vala = 5;
//  libconfig::Setting &ga = root.add("SomeGroup", libconfig::Setting::TypeGroup);
//  libconfig::Setting &ti = ga.add("fps", libconfig::Setting::TypeInt);
//  ti = 27;

//  libconfig::Setting &arr = ga.add("array", libconfig::Setting::TypeArray);
//  for (int i = 0; i < 3; ++i)
//  {
//    libconfig::Setting &tmp = arr.add(libconfig::Setting::TypeInt);
//    tmp = i;
//  }

//  libconfig::Setting &lst = ga.add("MyList", libconfig::Setting::TypeList);
//  for (int i = 0; i < 5; ++i)
//  {
//    libconfig::Setting &tmp = lst.add(libconfig::Setting::TypeInt);
//    tmp = i;
//  }

//  cfg.writeFile("fubar.txt");




  const auto config = vcp::config::LoadConfigParamsCpp("config.cfg");

  std::cout << "Int: " << config->GetInteger("an_int") << std::endl <<
               "String: " << config->GetString("some_string") << std::endl;

  std::cout << "Ints: " << std::endl;
  auto array_int = config->GetIntegerArray("ints");
  for (auto a : array_int)
    std::cout << a << ", ";
  std::cout << std::endl;

  std::cout << "Hexs: " << std::endl;
  auto array_hex = config->GetIntegerArray("hexs");
  for (auto a : array_hex)
    std::cout << a << " ";
  std::cout << std::endl;


  std::cout << "Strings: " << std::endl;
  auto array_str = config->GetStringArray("strings");
  for (auto a : array_str)
    std::cout << a << " ";
  std::cout << std::endl;

  std::cout << "All parameters inside 'a_group':" << std::endl;
  auto array_params = config->ListConfigGroupParameters("a_group");
  for (const auto &a : array_params)
    std::cout << "  " << a << std::endl;

  //std::cout << "Does key exist: 'foo' " << config->SettingExists("foo") << std::endl << "Now querying it... expect abort:" << std::endl;
  //config->GetInteger("foo");

  return 0;
}
