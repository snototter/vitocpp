#include "string_utils.h"
#include <gtest/gtest.h>
#include <string>

namespace pvt {
namespace utils {
namespace test {
TEST(StringUtils, EndsWith) {
  EXPECT_TRUE(string::EndsWith("foo", 'o'));
  EXPECT_TRUE(string::EndsWith("foo", "o"));
  EXPECT_TRUE(string::EndsWith("foo", "foo"));
  EXPECT_FALSE(string::EndsWith("foo", "foofoo"));
}

TEST(StringUtils, ToLower) {
  std::string str("");
  string::ToLower(str);
  EXPECT_EQ(str, "");

  str = "FoO";
  string::ToLower(str);
  EXPECT_EQ(str, "foo");

  str = "!343FoO§";
  string::ToLower(str);
  EXPECT_EQ(str, "!343foo§");
}

TEST(StringUtils, ToUpper) {
  std::string str("");
  string::ToUpper(str);
  EXPECT_EQ(str, "");

  str = "FoO";
  string::ToUpper(str);
  EXPECT_EQ(str, "FOO");

  str = "!343FoO§";
  string::ToUpper(str);
  EXPECT_EQ(str, "!343FOO§");
}

TEST(StringUtils, IsNumeric) {
  EXPECT_TRUE(string::IsNumeric("0"));
  EXPECT_TRUE(string::IsNumeric("-1"));
  EXPECT_TRUE(string::IsNumeric("42"));
  EXPECT_TRUE(string::IsNumeric(".01"));
  EXPECT_TRUE(string::IsNumeric("0.0093"));
  EXPECT_TRUE(string::IsNumeric("1e-3"));
  EXPECT_TRUE(string::IsNumeric("2e5"));
  EXPECT_TRUE(string::IsNumeric("0x01"));
  EXPECT_FALSE(string::IsNumeric("0a"));
}

TEST(StringUtils, Trim) {
  std::string str("");
  string::Trim(str);
  EXPECT_EQ(str, "");

  str = " \ta b\n";
  string::LTrim(str);
  EXPECT_EQ(str, "a b\n");
  string::Trim(str);
  EXPECT_EQ(str, "a b");

  str = "\t\n foo\n\t";
  string::RTrim(str);
  EXPECT_EQ(str, "\t\n foo");
  string::Trim(str);
  EXPECT_EQ(str, "foo");

  str = "\t\n foo\n\t";
  string::Trim(str);
  EXPECT_EQ(str, "foo");
}

TEST(StringUtils, Split) {
  std::vector<std::string> tokens = string::Split("",' ');
  EXPECT_EQ(tokens.size(), 0);

  tokens = string::Split("a b c d", ' ');
  EXPECT_EQ(tokens.size(), 4);
  EXPECT_EQ(tokens[0], "a");
  EXPECT_EQ(tokens[1], "b");
  EXPECT_EQ(tokens[2], "c");
  EXPECT_EQ(tokens[3], "d");

  tokens = string::Split("a;b c;d;", ';');
  EXPECT_EQ(tokens.size(), 3);
  EXPECT_EQ(tokens[0], "a");
  EXPECT_EQ(tokens[1], "b c");
  EXPECT_EQ(tokens[2], "d");
}

TEST(StringUtils, Replace) {
  const std::string str("FooFOOfooFOO");
  std::string rep = string::Replace(str, "foo", "bar");
  EXPECT_EQ(rep, "FooFOObarFOO");

  rep = string::Replace(str, "FOO", "bla");
  EXPECT_EQ(rep, "Fooblafoobla");
}
} // namespace test
} // namespace utils
} // namespace pvt
