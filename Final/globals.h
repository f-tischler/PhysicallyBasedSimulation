//
// Created by ivan on 17/01/18.
//

#ifndef PHYSICALLYBASEDSIMULATION_GLOBALS_H
#define PHYSICALLYBASEDSIMULATION_GLOBALS_H

constexpr int circle = 30;

extern int width;
extern int height;
constexpr double camera[] = {0.0, 1.0, 0.0, 1.0};

extern double xs;    /* Coordinates of mouse */
extern double ys;


static std::default_random_engine rng;

#endif //PHYSICALLYBASEDSIMULATION_GLOBALS_H
