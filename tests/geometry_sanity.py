#!/usr/bin/env python3
import math

def sphere_volume(r):
    return 4.0 / 3.0 * math.pi * r**3 if r > 0 else 0.0

def overlap(r, R, d):
    if r <= 0 or R <= 0 or d >= r + R:
        return 0.0
    if d <= abs(R-r) + 1e-12:
        return sphere_volume(min(r, R))
    term = r + R - d
    return math.pi * term**2 * (d*d + 2*d*(r+R) - 3*(r-R)**2) / (12*d)

def barrier(x, mu=0.02):
    if x <= 0: return 0.0
    if x < mu: return (mu - 0.5*x) * (x/mu)**3
    return x - 0.5*mu

assert abs(overlap(1.0, 1.0, 0.0) - sphere_volume(1.0)) < 1e-9
assert overlap(1.0, 1.0, 2.0) == 0.0
assert barrier(-1.0) == 0.0
assert abs(barrier(0.02) - 0.01) < 1e-12
for x in (0.001, 0.01, 0.019, 0.03):
    h = 1e-7
    numeric = (barrier(x+h) - barrier(x-h)) / (2*h)
    if x < 0.02:
        ratio = x / 0.02
        analytic = ratio*ratio*(3-2*ratio)
    else:
        analytic = 1.0
    assert abs(numeric-analytic) < 1e-5, (x, numeric, analytic)
print("Geometry sanity checks passed.")
