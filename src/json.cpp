// json.cpp — minimal JSON parser/serializer.
//
// Scope is deliberately the SDK's own needs: the flat calibration object and
// the flat safety-limits baseline. Numbers are written with the shortest
// round-trip representation (std::to_chars, same idea as Python's repr) so a
// file written here reads back identically in the Python SDK and vice versa.

#include "litegrip/json.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <system_error>

namespace litegrip::json {
namespace {

constexpr double kIntegralPrintLimit = 1e15;

void append_escaped(std::string& out, const std::string& text) {
  out.push_back('"');
  for (const char raw : text) {
    const auto ch = static_cast<unsigned char>(raw);
    switch (ch) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (ch < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
          out += buf;
        } else {
          out.push_back(raw);
        }
        break;
    }
  }
  out.push_back('"');
}

void append_number(std::string& out, double value) {
  if (!std::isfinite(value)) {
    // JSON has no NaN/Infinity. Emitting 0 is the least surprising choice for a
    // value that should never occur; callers validate their own fields.
    out += '0';
    return;
  }
  if (std::floor(value) == value && std::fabs(value) < kIntegralPrintLimit) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.0f", value);
    out += buf;
    return;
  }
  char buf[40];
  const auto result =
      std::to_chars(buf, buf + sizeof(buf), value);  // shortest round-trip
  if (result.ec == std::errc()) {
    out.append(buf, result.ptr);
  } else {
    std::snprintf(buf, sizeof(buf), "%.17g", value);
    out += buf;
  }
}

class Parser {
 public:
  Parser(const char* begin, const char* end) : cursor_(begin), end_(end) {}

  bool ok() const { return ok_; }

  Value parse_document() {
    skip_ws();
    Value value = parse_value();
    skip_ws();
    if (cursor_ != end_) {
      ok_ = false;  // trailing garbage
    }
    return value;
  }

 private:
  void skip_ws() {
    while (cursor_ != end_ &&
           (*cursor_ == ' ' || *cursor_ == '\t' || *cursor_ == '\n' ||
            *cursor_ == '\r')) {
      ++cursor_;
    }
  }

  bool consume(char expected) {
    if (cursor_ != end_ && *cursor_ == expected) {
      ++cursor_;
      return true;
    }
    return false;
  }

  bool looking_at(const char* literal) {
    const std::size_t length = std::strlen(literal);
    if (static_cast<std::size_t>(end_ - cursor_) < length) {
      return false;
    }
    return std::strncmp(cursor_, literal, length) == 0;
  }

  Value parse_value() {
    if (cursor_ == end_) {
      ok_ = false;
      return Value{};
    }
    switch (*cursor_) {
      case '{':
        return parse_object();
      case '[':
        return parse_array();
      case '"':
        return parse_string();
      case 't':
        if (looking_at("true")) {
          cursor_ += 4;
          return Value(true);
        }
        break;
      case 'f':
        if (looking_at("false")) {
          cursor_ += 5;
          return Value(false);
        }
        break;
      case 'n':
        if (looking_at("null")) {
          cursor_ += 4;
          return Value(nullptr);
        }
        break;
      default:
        break;
    }
    if (*cursor_ == '-' || (*cursor_ >= '0' && *cursor_ <= '9')) {
      return parse_number();
    }
    ok_ = false;
    return Value{};
  }

  Value parse_number() {
    const char* start = cursor_;
    if (cursor_ != end_ && *cursor_ == '-') {
      ++cursor_;
    }
    while (cursor_ != end_ && *cursor_ >= '0' && *cursor_ <= '9') {
      ++cursor_;
    }
    if (cursor_ != end_ && *cursor_ == '.') {
      ++cursor_;
      while (cursor_ != end_ && *cursor_ >= '0' && *cursor_ <= '9') {
        ++cursor_;
      }
    }
    if (cursor_ != end_ && (*cursor_ == 'e' || *cursor_ == 'E')) {
      ++cursor_;
      if (cursor_ != end_ && (*cursor_ == '+' || *cursor_ == '-')) {
        ++cursor_;
      }
      while (cursor_ != end_ && *cursor_ >= '0' && *cursor_ <= '9') {
        ++cursor_;
      }
    }
    const std::string text(start, cursor_);
    char* parse_end = nullptr;
    const double value = std::strtod(text.c_str(), &parse_end);
    if (parse_end == text.c_str()) {
      ok_ = false;
      return Value{};
    }
    return Value(value);
  }

  Value parse_string() {
    if (!consume('"')) {
      ok_ = false;
      return Value{};
    }
    std::string out;
    while (cursor_ != end_) {
      const char ch = *cursor_++;
      if (ch == '"') {
        return Value(std::move(out));
      }
      if (ch != '\\') {
        out.push_back(ch);
        continue;
      }
      if (cursor_ == end_) {
        break;
      }
      const char escape = *cursor_++;
      switch (escape) {
        case '"':
          out.push_back('"');
          break;
        case '\\':
          out.push_back('\\');
          break;
        case '/':
          out.push_back('/');
          break;
        case 'b':
          out.push_back('\b');
          break;
        case 'f':
          out.push_back('\f');
          break;
        case 'n':
          out.push_back('\n');
          break;
        case 'r':
          out.push_back('\r');
          break;
        case 't':
          out.push_back('\t');
          break;
        case 'u': {
          if (end_ - cursor_ < 4) {
            ok_ = false;
            return Value{};
          }
          unsigned int code = 0;
          for (int i = 0; i < 4; ++i) {
            const char digit = *cursor_++;
            code <<= 4;
            if (digit >= '0' && digit <= '9') {
              code |= static_cast<unsigned int>(digit - '0');
            } else if (digit >= 'a' && digit <= 'f') {
              code |= static_cast<unsigned int>(digit - 'a' + 10);
            } else if (digit >= 'A' && digit <= 'F') {
              code |= static_cast<unsigned int>(digit - 'A' + 10);
            } else {
              ok_ = false;
              return Value{};
            }
          }
          // Encode as UTF-8 (surrogate pairs are passed through unpaired; the
          // SDK never emits them).
          if (code < 0x80) {
            out.push_back(static_cast<char>(code));
          } else if (code < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (code >> 6)));
            out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
          } else {
            out.push_back(static_cast<char>(0xE0 | (code >> 12)));
            out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
          }
          break;
        }
        default:
          ok_ = false;
          return Value{};
      }
    }
    ok_ = false;
    return Value{};
  }

  Value parse_array() {
    Value array = Value::make_array();
    if (!consume('[')) {
      ok_ = false;
      return array;
    }
    skip_ws();
    if (consume(']')) {
      return array;
    }
    while (true) {
      skip_ws();
      array.push_back(parse_value());
      if (!ok_) {
        return array;
      }
      skip_ws();
      if (consume(',')) {
        continue;
      }
      if (consume(']')) {
        return array;
      }
      ok_ = false;
      return array;
    }
  }

  Value parse_object() {
    Value object = Value::make_object();
    if (!consume('{')) {
      ok_ = false;
      return object;
    }
    skip_ws();
    if (consume('}')) {
      return object;
    }
    while (true) {
      skip_ws();
      if (cursor_ == end_ || *cursor_ != '"') {
        ok_ = false;
        return object;
      }
      Value key = parse_string();
      if (!ok_) {
        return object;
      }
      skip_ws();
      if (!consume(':')) {
        ok_ = false;
        return object;
      }
      skip_ws();
      Value value = parse_value();
      if (!ok_) {
        return object;
      }
      object.set(key.as_string(), std::move(value));
      skip_ws();
      if (consume(',')) {
        continue;
      }
      if (consume('}')) {
        return object;
      }
      ok_ = false;
      return object;
    }
  }

  const char* cursor_;
  const char* end_;
  bool ok_ = true;
};

}  // namespace

Value Value::make_array() {
  Value value;
  value.type_ = Type::kArray;
  return value;
}

Value Value::make_object() {
  Value value;
  value.type_ = Type::kObject;
  return value;
}

bool Value::as_bool(bool fallback) const noexcept {
  return type_ == Type::kBool ? bool_ : fallback;
}

double Value::as_number(double fallback) const noexcept {
  return type_ == Type::kNumber ? number_ : fallback;
}

std::string Value::as_string(const std::string& fallback) const {
  return type_ == Type::kString ? string_ : fallback;
}

const Value* Value::find(const std::string& key) const noexcept {
  if (type_ != Type::kObject) {
    return nullptr;
  }
  for (const auto& entry : object_) {
    if (entry.first == key) {
      return &entry.second;
    }
  }
  return nullptr;
}

bool Value::contains(const std::string& key) const noexcept {
  return find(key) != nullptr;
}

double Value::get_number(const std::string& key, double fallback) const noexcept {
  const Value* value = find(key);
  return value != nullptr ? value->as_number(fallback) : fallback;
}

int Value::get_int(const std::string& key, int fallback) const noexcept {
  const Value* value = find(key);
  if (value == nullptr || value->type() != Type::kNumber) {
    return fallback;
  }
  return static_cast<int>(value->as_number(static_cast<double>(fallback)));
}

bool Value::get_bool(const std::string& key, bool fallback) const noexcept {
  const Value* value = find(key);
  return value != nullptr ? value->as_bool(fallback) : fallback;
}

std::string Value::get_string(const std::string& key,
                              const std::string& fallback) const {
  const Value* value = find(key);
  return value != nullptr ? value->as_string(fallback) : fallback;
}

void Value::set(const std::string& key, Value value) {
  if (type_ != Type::kObject) {
    type_ = Type::kObject;
    object_.clear();
  }
  for (auto& entry : object_) {
    if (entry.first == key) {
      entry.second = std::move(value);
      return;
    }
  }
  object_.emplace_back(key, std::move(value));
}

std::size_t Value::size() const noexcept {
  if (type_ == Type::kArray) {
    return array_.size();
  }
  if (type_ == Type::kObject) {
    return object_.size();
  }
  return 0;
}

void Value::push_back(Value value) {
  if (type_ != Type::kArray) {
    type_ = Type::kArray;
    array_.clear();
  }
  array_.push_back(std::move(value));
}

const Value& Value::operator[](std::size_t index) const {
  return array_.at(index);
}

void Value::dump_to(std::string& out, int indent, int depth) const {
  const bool pretty = indent > 0;
  const std::string pad =
      pretty ? std::string(static_cast<std::size_t>(indent * (depth + 1)), ' ')
             : std::string();
  const std::string close_pad =
      pretty ? std::string(static_cast<std::size_t>(indent * depth), ' ')
             : std::string();

  switch (type_) {
    case Type::kNull:
      out += "null";
      return;
    case Type::kBool:
      out += bool_ ? "true" : "false";
      return;
    case Type::kNumber:
      append_number(out, number_);
      return;
    case Type::kString:
      append_escaped(out, string_);
      return;
    case Type::kArray: {
      if (array_.empty()) {
        out += "[]";
        return;
      }
      out.push_back('[');
      for (std::size_t i = 0; i < array_.size(); ++i) {
        if (i != 0) {
          out.push_back(',');
        }
        if (pretty) {
          out.push_back('\n');
          out += pad;
        }
        array_[i].dump_to(out, indent, depth + 1);
      }
      if (pretty) {
        out.push_back('\n');
        out += close_pad;
      }
      out.push_back(']');
      return;
    }
    case Type::kObject: {
      if (object_.empty()) {
        out += "{}";
        return;
      }
      out.push_back('{');
      for (std::size_t i = 0; i < object_.size(); ++i) {
        if (i != 0) {
          out.push_back(',');
        }
        if (pretty) {
          out.push_back('\n');
          out += pad;
        }
        append_escaped(out, object_[i].first);
        out.push_back(':');
        if (pretty) {
          out.push_back(' ');
        }
        object_[i].second.dump_to(out, indent, depth + 1);
      }
      if (pretty) {
        out.push_back('\n');
        out += close_pad;
      }
      out.push_back('}');
      return;
    }
  }
}

std::string Value::dump(int indent) const {
  std::string out;
  dump_to(out, indent, 0);
  return out;
}

std::optional<Value> Value::parse(const std::string& text) {
  Parser parser(text.data(), text.data() + text.size());
  Value value = parser.parse_document();
  if (!parser.ok()) {
    return std::nullopt;
  }
  return value;
}

std::optional<Value> Value::parse_file(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return std::nullopt;
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return parse(buffer.str());
}

bool Value::write_file(const std::string& path, int indent) const {
  // Create parent directories, mirroring the Python SDK (which used
  // os.makedirs(parent, exist_ok=True)).
  const std::size_t slash = path.find_last_of('/');
  if (slash != std::string::npos && slash > 0) {
    const std::string parent = path.substr(0, slash);
    std::string accumulated;
    std::size_t start = 0;
    while (start <= parent.size()) {
      const std::size_t next = parent.find('/', start);
      const std::string component =
          parent.substr(start, next == std::string::npos ? std::string::npos
                                                         : next - start);
      if (!component.empty()) {
        accumulated += "/" + component;
        ::mkdir(accumulated.c_str(), 0755);
      }
      if (next == std::string::npos) {
        break;
      }
      start = next + 1;
    }
  }

  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    return false;
  }
  output << dump(indent);
  return output.good();
}

}  // namespace litegrip::json
