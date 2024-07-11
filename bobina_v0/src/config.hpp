#pragma once

struct Config {
  uint8_t startIndex : 2;     // Va de 0 a 3, mirar starts
  uint8_t strategyIndex : 1;  // Va de 0 a 1, mirar strategies
  uint8_t escapeIndex : 2;    // Va de 0 a 3, mirar escapes
  uint8_t velocity : 3;       //  Va de 0 a 7, 7 es LPM QUE RAPIDO, 0 es abuelina
} __attribute__((packed)) config;