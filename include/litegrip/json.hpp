// litegrip/json.hpp — minimal JSON value/parser/serializer (zero dependency).
//
// The SDK must stay dependency-free (see PLAN-litegrip-cpp.md D7), and its
// JSON needs are tiny and fixed: read/write the flat calibration object and
// read the flat safety-limits baseline. So instead of pulling in
// nlohmann/json this implements exactly what is needed, and no more.
//
// Supported: null, bool, number (double), string, array, object.
// Not supported (deliberately): UTF-8 escape validation beyond \uXXXX,
// BigInt/precision beyond double, comments, trailing commas.

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace litegrip::json {

/// A JSON value. Objects keep insertion order via the ordered key vector.
class Value {
 public:
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

  Value() = default;
  Value(std::nullptr_t) {}
  Value(bool b) : type_(Type::kBool), bool_(b) {}
  Value(double n) : type_(Type::kNumber), number_(n) {}
  Value(int n) : type_(Type::kNumber), number_(static_cast<double>(n)) {}
  Value(std::string s) : type_(Type::kString), string_(std::move(s)) {}
  Value(const char* s) : type_(Type::kString), string_(s) {}

  static Value make_array();
  static Value make_object();

  Type type() const noexcept { return type_; }
  bool is_null() const noexcept { return type_ == Type::kNull; }
  bool is_bool() const noexcept { return type_ == Type::kBool; }
  bool is_number() const noexcept { return type_ == Type::kNumber; }
  bool is_string() const noexcept { return type_ == Type::kString; }
  bool is_array() const noexcept { return type_ == Type::kArray; }
  bool is_object() const noexcept { return type_ == Type::kObject; }

  // ── scalar accessors (never throw; fall back to the given default) ────
  bool as_bool(bool fallback = false) const noexcept;
  double as_number(double fallback = 0.0) const noexcept;
  std::string as_string(const std::string& fallback = "") const;

  // ── object access ────────────────────────────────────────────────────
  bool contains(const std::string& key) const noexcept;

  /// Member access; returns nullopt when absent or when this is not an object.
  const Value* find(const std::string& key) const noexcept;

  /// Read a member with a typed default and an optional range check. These are
  /// what the calibration / safety loaders use.
  double get_number(const std::string& key, double fallback) const noexcept;
  int get_int(const std::string& key, int fallback) const noexcept;
  bool get_bool(const std::string& key, bool fallback) const noexcept;
  std::string get_string(const std::string& key, const std::string& fallback) const;

  void set(const std::string& key, Value value);
  void set(const std::string& key, double value) { set(key, Value(value)); }
  void set(const std::string& key, int value) { set(key, Value(value)); }
  void set(const std::string& key, bool value) { set(key, Value(value)); }
  void set(const std::string& key, const char* value) { set(key, Value(value)); }

  // ── array access ─────────────────────────────────────────────────────
  std::size_t size() const noexcept;
  void push_back(Value value);
  /// Bounds-checked; throws std::out_of_range when out of range.
  const Value& operator[](std::size_t index) const;

  /// Serialize. `indent <= 0` produces compact output.
  std::string dump(int indent = 2) const;

  // ── parsing ──────────────────────────────────────────────────────────
  /// Parse `text`; nullopt when malformed.
  static std::optional<Value> parse(const std::string& text);

  /// Read + parse a file; nullopt when missing or malformed.
  static std::optional<Value> parse_file(const std::string& path);

  /// Write `dump(indent)` to `path`, creating parent directories.
  /// Returns false on I/O failure.
  bool write_file(const std::string& path, int indent = 2) const;

 private:
  void dump_to(std::string& out, int indent, int depth) const;

  Type type_ = Type::kNull;
  bool bool_ = false;
  double number_ = 0.0;
  std::string string_;
  std::vector<Value> array_;
  std::vector<std::pair<std::string, Value>> object_;
};

}  // namespace litegrip::json
