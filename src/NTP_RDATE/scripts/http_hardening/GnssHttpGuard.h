#pragma once

#include <Arduino.h>

// All limits apply before application handlers, including unmatched/public URLs.
// Uploads stream through the framework buffer; these limits do not allocate a body.
class GnssHttpGuard {
public:
  static constexpr uint32_t MaxBodyBytes = 2U * 1024U * 1024U;
  static constexpr uint32_t MaxPlainBytes = 4096;
  static constexpr uint32_t MaxFieldBytes = 1024;
  static constexpr uint32_t MaxFieldsBytes = 4096;
  static constexpr uint32_t IdleMs = 1000;

  void beginHeaders() {
    started_ = lastByte_ = millis();
    deadline_ = 5000;
    remaining_ = 8192;
    failed_ = false;
  }

  bool beginBody(uint32_t length, bool multipart) {
    if (length > (multipart ? MaxBodyBytes : MaxPlainBytes)) return fail();
    started_ = lastByte_ = millis();
    deadline_ = multipart ? 60000 : 5000;
    remaining_ = length;
    return !failed_;
  }

  uint32_t remaining() const { return remaining_; }

  // Content-Length must be an unsigned decimal without overflow or ambiguity.
  static bool contentLength(const String& text, uint32_t& value) {
    if (!text.length()) return false;
    value = 0;
    for (size_t i = 0; i < text.length(); ++i) {
      char ch = text[i];
      if (ch < '0' || ch > '9') return false;
      unsigned digit = static_cast<unsigned>(ch - '0');
      if (value > (MaxBodyBytes - digit) / 10U) return false;
      value = value * 10U + digit;
    }
    return true;
  }

  template <typename Client> int readByte(Client& client) {
    if (failed_ || !remaining_) { fail(); return -1; }
    for (;;) {
      uint32_t now = millis();
      if (uint32_t(now - started_) >= deadline_ ||
          uint32_t(now - lastByte_) >= IdleMs) {
        fail(); return -1;
      }
      int ch = client.read();
      if (ch >= 0) {
        --remaining_;
        lastByte_ = now;
        return ch;
      }
      if (!client.connected()) { fail(); return -1; }
      delay(1);
    }
  }

  template <typename Client> bool readLine(Client& client, String& line) {
    line = "";
    for (;;) {
      int ch = readByte(client);
      if (ch < 0) return false;
      if (ch == '\r') return readByte(client) == '\n' || fail();
      if (ch == '\n' || line.length() >= 1024 || !line.concat(char(ch))) return fail();
    }
  }

  template <typename Client> size_t readBuffer(Client& client, uint8_t* output, size_t size) {
    size_t count = 0;
    while (count < size) {
      int ch = readByte(client);
      if (ch < 0) break;
      output[count++] = uint8_t(ch);
    }
    return count;
  }

  template <typename Client> bool readText(Client& client, uint32_t length, String& text) {
    text = "";
    if (length > MaxPlainBytes || !text.reserve(length)) return fail();
    for (uint32_t i = 0; i < length; ++i) {
      int ch = readByte(client);
      if (ch < 0 || ch == 0 || !text.concat(char(ch))) return fail();
    }
    return true;
  }

private:
  bool fail() { failed_ = true; return false; }
  uint32_t started_ = 0, lastByte_ = 0, deadline_ = 0, remaining_ = 0;
  bool failed_ = false;
};
