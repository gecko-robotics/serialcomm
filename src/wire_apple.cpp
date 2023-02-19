#include "serialcomm/wire.hpp"

#if defined(__APPLE__)

TwoWire::TwoWire() {}

TwoWire::~TwoWire() {}

void TwoWire::set(uint8_t address) {}

bool TwoWire::read(const uint8_t reg, const uint8_t count,
                   uint8_t *const data) {
  return true;
}

bool TwoWire::write(const uint8_t reg, const uint8_t data) { return true; }
#endif