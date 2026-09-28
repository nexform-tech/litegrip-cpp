// test_json.cpp — the minimal JSON parser/serializer.
//
// The SDK's files are read back by the Python SDK (and vice versa) during
// parallel validation, so both structure and number formatting matter.

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>

#include "litegrip/json.hpp"

using litegrip::json::Value;

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

void check_str(const std::string& got, const std::string& want,
               const char* what) {
  if (got != want) {
    std::cerr << "FAIL: " << what << "\n  got:  " << got << "\n  want: " << want
              << "\n";
    ++g_failures;
  }
}

}  // namespace

int main() {
  // ── scalars ───────────────────────────────────────────────────────────
  {
    const auto value = Value::parse("42");
    check(value.has_value() && value->is_number(), "parse integer");
    check(value.has_value() && value->as_number() == 42.0, "integer value");
  }
  {
    const auto value = Value::parse("-1.5e2");
    check(value.has_value() && value->is_number(), "parse exponent");
    if (value.has_value()) {
      check(std::fabs(value->as_number() - (-150.0)) < 1e-9, "exponent value");
    }
  }
  {
    const auto value = Value::parse("true");
    check(value.has_value() && value->is_bool() && value->as_bool(),
          "parse true");
    const auto other = Value::parse("false");
    check(other.has_value() && other->is_bool() && !other->as_bool(),
          "parse false");
    const auto nil = Value::parse("null");
    check(nil.has_value() && nil->is_null(), "parse null");
  }

  // ── strings and escapes ───────────────────────────────────────────────
  {
    const auto value = Value::parse("\"hello\"");
    check(value.has_value() && value->is_string(), "parse string");
    check(value.has_value() && value->as_string() == "hello", "string value");
  }
  {
    const auto value =
        Value::parse("\"a\\\"b\\\\c\\nd\\te\\u0041\"");
    check(value.has_value(), "parse escaped string");
    if (value.has_value()) {
      check_str(value->as_string(), "a\"b\\c\nd\teA", "escape decoding");
    }
  }
  {
    const auto value = Value::parse("\"\\u00e9\"");
    check(value.has_value(), "parse \\u with 2-byte UTF-8");
    if (value.has_value()) {
      check_str(value->as_string(), "\xc3\xa9", "2-byte UTF-8 encoding");
    }
  }

  // ── objects and arrays ────────────────────────────────────────────────
  {
    const auto value = Value::parse("{\"a\": 1, \"b\": \"two\", \"c\": true}");
    check(value.has_value() && value->is_object(), "parse object");
    if (value.has_value()) {
      check(value->contains("a") && value->contains("b") && value->contains("c"),
            "object keys present");
      check(!value->contains("zzz"), "absent key reports false");
      check(value->find("zzz") == nullptr, "absent key yields no value");
      check(value->get_int("a", 0) == 1, "get_int");
      check_str(value->get_string("b", ""), "two", "get_string");
      check(value->get_bool("c", false), "get_bool");
      // Missing keys fall back rather than throwing.
      check(value->get_int("zzz", 7) == 7, "get_int default");
      check(value->get_number("zzz", 2.5) == 2.5, "get_number default");
      check(value->get_bool("zzz", true), "get_bool default");
    }
  }
  {
    const auto value = Value::parse("[1, 2, [3, 4], {\"k\": 5}]");
    check(value.has_value() && value->is_array(), "parse nested array");
    if (value.has_value()) {
      check(value->size() == 4, "array size");
      check(value->get_number("", 0.0) == 0.0, "get_* on an array defaults");
      check((*value)[0].as_number() == 1.0, "array index 0");
      check((*value)[2].is_array() && (*value)[2].size() == 2, "nested array");
      check((*value)[3].is_object(), "object inside array");
    }
  }

  // ── malformed input is rejected, not guessed at ───────────────────────
  for (const char* bad : {"", "{", "}", "[1,]", "{\"a\"}", "{\"a\":}", "tru",
                          "nul", "123abc", "\"unterminated", "[", "{\"a\":1,}"}) {
    check(!Value::parse(bad).has_value(), "malformed input rejected");
  }

  // ── dump ──────────────────────────────────────────────────────────────
  {
    Value object = Value::make_object();
    object.set("channel", "can0");
    object.set("can_id", 8);
    object.set("rad_to_mm", 0.114);
    object.set("canfd_mode", false);
    check_str(object.dump(0),
              "{\"channel\":\"can0\",\"can_id\":8,\"rad_to_mm\":0.114,"
              "\"canfd_mode\":false}",
              "compact dump");
  }
  {
    // Shortest round-trip number formatting, so files stay diffable and match
    // what Python's json.dump writes.
    check_str(Value(0.114).dump(0), "0.114", "0.114 formats short");
    check_str(Value(65.0231).dump(0), "65.0231", "65.0231 formats short");
    check_str(Value(8.0).dump(0), "8", "integral double prints without .0");
    check_str(Value(-0.01).dump(0), "-0.01", "small negative formats short");
  }
  {
    Value array = Value::make_array();
    array.push_back(Value(1));
    array.push_back(Value("x"));
    array.push_back(Value(true));
    check_str(array.dump(0), "[1,\"x\",true]", "array dump");
  }
  {
    Value empty_object = Value::make_object();
    Value empty_array = Value::make_array();
    check_str(empty_object.dump(0), "{}", "empty object dump");
    check_str(empty_array.dump(0), "[]", "empty array dump");
  }

  // ── round-trip ────────────────────────────────────────────────────────
  {
    const std::string text =
        "{\"a\":[1,2.5,\"s\"],\"b\":{\"c\":null},\"d\":true}";
    const auto parsed = Value::parse(text);
    check(parsed.has_value(), "round-trip parse");
    if (parsed.has_value()) {
      check_str(parsed->dump(0), text, "round-trip is byte-identical");
    }
  }

  // ── file I/O (creates parent directories, like the Python SDK) ────────
  {
    const std::string path =
        "/tmp/litegrip_json_test/nested/dir/sample.json";
    ::unlink(path.c_str());

    Value document = Value::make_object();
    document.set("can_id", 8);
    document.set("rad_to_mm", 74.8);
    document.set("motor_type", "DM4310");
    check(document.write_file(path), "write_file creates parents");

    const auto reloaded = Value::parse_file(path);
    check(reloaded.has_value(), "parse_file reads it back");
    if (reloaded.has_value()) {
      check(reloaded->get_int("can_id", 0) == 8, "reloaded can_id");
      check(std::fabs(reloaded->get_number("rad_to_mm", 0.0) - 74.8) < 1e-12,
            "reloaded rad_to_mm");
      check_str(reloaded->get_string("motor_type", ""), "DM4310",
                "reloaded motor_type");
    }
  }
  check(!Value::parse_file("/tmp/litegrip_json_test/does/not/exist.json")
             .has_value(),
        "missing file yields no value");

  if (g_failures != 0) {
    std::cerr << g_failures << " json check(s) failed\n";
    return 1;
  }
  std::cout << "json checks OK\n";
  return 0;
}
