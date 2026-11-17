#pragma once

namespace green2test {

// Only command lines are actionable. Echoed diagnostics and malformed input
// must not produce replies that can feed back into RX again.
class ConsoleLine {
 public:
  const char* push(char c) {
    if (c == '\r' || c == '\n') {
      const bool command = !discard_ && length_ >= 2 && buffer_[0] == '@' &&
          buffer_[1] == 'g' && (length_ == 2 || buffer_[2] == ' ');
      buffer_[length_] = 0;
      length_ = 0;
      discard_ = false;
      return command ? buffer_ : nullptr;
    }
    if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) > 0x7e)
      discard_ = true;
    else if (length_ < sizeof(buffer_) - 1) buffer_[length_++] = c;
    else discard_ = true;
    return nullptr;
  }

 private:
  char buffer_[96] = {};
  unsigned length_ = 0;
  bool discard_ = false;
};

}  // namespace green2test
