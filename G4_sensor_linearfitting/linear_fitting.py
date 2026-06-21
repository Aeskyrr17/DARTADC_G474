#!/usr/bin/env python3
# Fit force_kg = k * adc + b from multiple calibration points.

from math import sqrt
from collections import defaultdict

SENSOR_NAME = "L"  # "L" or "R"

# Fill your calibration data here: (force_kg, adc)
# You can put repeated measurements directly; set AVERAGE_SAME_FORCE below.
CALIB_POINTS = [
    # (0.0, 32767.0),
    # (20.0, 39320.0),
    # (50.0, 49151.0),
    # (100.0, 65535.0),
]

AVERAGE_SAME_FORCE = True


def read_points_interactive():
    points = []
    print("Input calibration points, one per line: force_kg adc")
    print("Example: 50 49151")
    print("Press Enter on an empty line to calculate.\n")

    while True:
        line = input("> ").strip()
        if not line:
            break

        line = line.replace(",", " ")
        parts = line.split()
        if len(parts) != 2:
            print("Invalid line. Use: force_kg adc")
            continue

        try:
            force_kg = float(parts[0])
            adc = float(parts[1])
        except ValueError:
            print("Invalid number.")
            continue

        points.append((force_kg, adc))

    return points


def average_same_force(points):
    grouped = defaultdict(list)
    for force_kg, adc in points:
        grouped[force_kg].append(adc)

    averaged = []
    for force_kg in sorted(grouped.keys()):
        adcs = grouped[force_kg]
        averaged.append((force_kg, sum(adcs) / len(adcs)))

    return averaged


def fit_linear_force_from_adc(points):
    # y = kx + b
    # x = adc, y = force_kg
    n = len(points)
    if n < 2:
        raise ValueError("Need at least 2 calibration points.")

    sum_x = sum(adc for _, adc in points)
    sum_y = sum(force_kg for force_kg, _ in points)
    sum_xx = sum(adc * adc for _, adc in points)
    sum_xy = sum(adc * force_kg for force_kg, adc in points)

    denom = n * sum_xx - sum_x * sum_x
    if abs(denom) < 1e-12:
        raise ValueError("All ADC values are too close; cannot fit k and b.")

    k = (n * sum_xy - sum_x * sum_y) / denom
    b = (sum_y - k * sum_x) / n
    return k, b


def calc_report(points, k, b):
    errors = []
    y_mean = sum(force_kg for force_kg, _ in points) / len(points)

    ss_res = 0.0
    ss_tot = 0.0

    for force_kg, adc in points:
        pred = k * adc + b
        err = pred - force_kg
        errors.append(err)
        ss_res += err * err
        ss_tot += (force_kg - y_mean) * (force_kg - y_mean)

    rmse = sqrt(ss_res / len(points))
    max_abs_error = max(abs(e) for e in errors)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0

    return rmse, max_abs_error, r2


def main():
    points = CALIB_POINTS
    if not points:
        points = read_points_interactive()

    if AVERAGE_SAME_FORCE:
        points = average_same_force(points)

    k, b = fit_linear_force_from_adc(points)
    rmse, max_abs_error, r2 = calc_report(points, k, b)

    sensor = SENSOR_NAME.upper()

    print("\nCalibration points used:")
    for force_kg, adc in points:
        pred = k * adc + b
        print(f"  force={force_kg:10.4f} kg, adc={adc:12.4f}, fit={pred:10.4f} kg, error={pred - force_kg:+.4f} kg")

    print("\nResult:")
    print(f"  force_kg = k * adc + b")
    print(f"  k = {k:.10f}f")
    print(f"  b = {b:.10f}f")

    print("\nPaste into main.cpp:")
    print(f"#define FORCE_SENSOR_{sensor}_LINEAR_K {k:.10f}f")
    print(f"#define FORCE_SENSOR_{sensor}_LINEAR_B {b:.10f}f")

    print("\nFit quality:")
    print(f"  RMSE          = {rmse:.6f} kg")
    print(f"  Max abs error = {max_abs_error:.6f} kg")
    print(f"  R^2           = {r2:.8f}")


if __name__ == "__main__":
    main()