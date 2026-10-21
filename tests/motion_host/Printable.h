#pragma once
#include "Arduino.h"
class Printable { public: virtual size_t printTo(Print&) const = 0; virtual ~Printable() = default; };
