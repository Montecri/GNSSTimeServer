#ifndef NMEA_RMC_CLOCK_H
#define NMEA_RMC_CLOCK_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// TinyGPSPlus is still used for navigation/display data. Clock synchronization needs a
// separate, complete RMC record: TinyGPSPlus can refresh an old date when a date field is empty.
// No allocation or Arduino dependencies, so these acceptance rules can also run in host tests.
class NmeaRmcClock
{
public:
  struct Sample
  {
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint32_t startedMicros, committedMicros, committedMillis;
    bool valid;
  };

  NmeaRmcClock() : length_(0), collecting_(false), startedMicros_(0)
  {
    memset(samples_, 0, sizeof(samples_));
  }

  void encode(char c, uint32_t nowMicros, uint32_t nowMillis)
  {
    if (c == '$')
    {
      length_ = 0;
      collecting_ = true;
      startedMicros_ = nowMicros;
      return;
    }
    if (!collecting_)
      return;
    if (c == '\r' || c == '\n')
    {
      collecting_ = false;
      line_[length_] = '\0';
      commit(nowMicros, nowMillis);
    }
    else if (c < ' ' || c > '~')
      collecting_ = false;
    else if (length_ < sizeof(line_) - 1)
      line_[length_++] = c;
    else
      collecting_ = false; // discard oversized sentences until the next '$'
  }

  // Receiver contract: an RMC describes the UTC second containing its transmission; the
  // following PPS begins UTC+1. Require the entire sentence to precede that edge and to lie
  // inside its preceding second. Other receiver PPS/NMEA conventions require configuration.
  // Keep the preceding record as well, because the parser may receive another RMC after the
  // target edge before the synchronization task gets to run.
  bool sampleForEdge(uint32_t edgeMicros, uint32_t nowMillis, uint32_t maxAgeMillis,
                     uint32_t minLeadMicros, uint32_t maxLeadMicros,
                     uint16_t minYear, uint16_t maxYear, Sample &result) const
  {
    for (size_t i = 0; i < 2; ++i)
    {
      const Sample &sample = samples_[i];
      uint32_t lead = edgeMicros - sample.committedMicros;
      uint32_t startLead = edgeMicros - sample.startedMicros;
      if (sample.valid && sample.year >= minYear && sample.year <= maxYear &&
          (uint32_t)(nowMillis - sample.committedMillis) < maxAgeMillis &&
          lead >= minLeadMicros && lead <= maxLeadMicros &&
          startLead >= lead && startLead <= maxLeadMicros)
      {
        result = sample;
        return true;
      }
    }
    return false;
  }

private:
  char line_[128];
  size_t length_;
  bool collecting_;
  uint32_t startedMicros_;
  Sample samples_[2];

  static int hexValue(char c)
  {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  }

  static bool decimalDigits(const char *value, size_t count)
  {
    for (size_t i = 0; i < count; ++i)
      if (value[i] < '0' || value[i] > '9') return false;
    return true;
  }

  static uint8_t decimalPair(const char *value)
  {
    return (uint8_t)((value[0] - '0') * 10 + value[1] - '0');
  }

  void commit(uint32_t nowMicros, uint32_t nowMillis)
  {
    char *checksum = strchr(line_, '*');
    if (!checksum || strlen(checksum) != 3)
      return;
    int high = hexValue(checksum[1]), low = hexValue(checksum[2]);
    if (high < 0 || low < 0)
      return;
    uint8_t parity = 0;
    for (const char *p = line_; p != checksum; ++p)
      parity ^= (uint8_t)*p;
    if (parity != (uint8_t)((high << 4) | low))
      return;
    *checksum = '\0';

    // Splitting in place preserves empty fields (strtok would hide them).
    char *fields[10];
    size_t fieldCount = 1;
    fields[0] = line_;
    for (char *p = line_; *p && fieldCount < 10; ++p)
      if (*p == ',')
      {
        *p = '\0';
        fields[fieldCount++] = p + 1;
      }
    if (fieldCount != 10)
      return;
    char *dateEnd = strchr(fields[9], ',');
    if (dateEnd) *dateEnd = '\0';

    // Accept the standard two-letter talkers, including GN for multi-constellation receivers.
    if (strlen(fields[0]) != 5 || fields[0][0] < 'A' || fields[0][0] > 'Z' ||
        fields[0][1] < 'A' || fields[0][1] > 'Z' || strcmp(fields[0] + 2, "RMC") != 0 ||
        strcmp(fields[2], "A") != 0 || strlen(fields[9]) != 6 || !decimalDigits(fields[9], 6))
      return;
    size_t timeLength = strlen(fields[1]);
    if (timeLength < 6 || !decimalDigits(fields[1], 6))
      return;
    if (timeLength > 6 &&
        (fields[1][6] != '.' || timeLength == 7 || !decimalDigits(fields[1] + 7, timeLength - 7)))
      return;

    Sample candidate = {};
    candidate.hour = decimalPair(fields[1]);
    candidate.minute = decimalPair(fields[1] + 2);
    candidate.second = decimalPair(fields[1] + 4);
    candidate.day = decimalPair(fields[9]);
    candidate.month = decimalPair(fields[9] + 2);
    candidate.year = 2000 + decimalPair(fields[9] + 4);
    if (candidate.hour > 23 || candidate.minute > 59 || candidate.second > 59 ||
        candidate.month < 1 || candidate.month > 12 || candidate.day < 1)
      return;
    static const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t days = monthDays[candidate.month - 1];
    bool leapYear = candidate.year % 4 == 0 && (candidate.year % 100 != 0 || candidate.year % 400 == 0);
    if (candidate.month == 2 && leapYear) ++days;
    if (candidate.day > days)
      return;
    candidate.startedMicros = startedMicros_;
    candidate.committedMicros = nowMicros;
    candidate.committedMillis = nowMillis;
    candidate.valid = true;
    samples_[1] = samples_[0];
    samples_[0] = candidate;
  }
};

#endif
