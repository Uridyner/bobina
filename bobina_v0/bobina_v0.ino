#include "src/starts.hpp"
#include "src/strategies.hpp"
#include "src/escapes.hpp"
#include "src/config.hpp"

void (*starts[4])() = {
  startDoNothing,
  startGoFoward,
  startCurveLeft,
  startCurveRight
};

void (*strategies[2])() = {
  strategySpinLeft,
  strategySpinRight
};

void (*escapes[4])() = {
  escapeBackwards,
  escapeCurvedLeft,
  escapeCurvedRight,
  escapeBackwards
};

void setup() {
  // set up config default values
  config.startIndex = 2;
  config.strategyIndex = 0;
  config.escapeIndex = 0;
  config.velocity = 6;
}

void loop() {
}
