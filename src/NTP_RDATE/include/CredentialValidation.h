#pragma once

#include <stddef.h>

namespace CredentialValidation
{
constexpr size_t MAX_SSID_BYTES = 32;
constexpr size_t MAX_ADMIN_BYTES = 63;

constexpr bool isHex(char ch)
{
  return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') ||
         (ch >= 'A' && ch <= 'F');
}

constexpr bool allHex(const char *value, size_t length)
{
  return length == 0 || (isHex(*value) && allHex(value + 1, length - 1));
}

// An empty submission means keep the stored password; the separate open-network
// checkbox explicitly clears it. A 64-byte hexadecimal value is a raw WPA PSK.
constexpr bool validStationPassword(const char *value, size_t length)
{
  return length == 0 || (length >= 8 && length <= 63) ||
         (length == 64 && allHex(value, length));
}

constexpr bool validApPassword(size_t length)
{
  return length >= 8 && length <= 63;
}

constexpr bool validAdminSubmission(size_t length)
{
  return length == 0 || (length >= 8 && length <= MAX_ADMIN_BYTES);
}
}
