"""
Grid-based ground-risk assessment for urban low-altitude UAV operations.

The implementation follows the paper's main workflow:
1. simulate a UAV failure descent and build an impact-position probability kernel;
2. combine the kernel with population density, exposure area and sheltering effect;
3. fuse no-fly, obstacle and special-area layers into a final risk map.

All grids are row-major: grid[y][x].
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import random
from typing import Iterable, List, Optional, Sequence, Tuple


NumberGrid = List[List[float]]
BoolGrid = List[List[bool]]
LevelGrid = List[List[int]]


class RiskLevel(IntEnum):
    SAFE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    FORBIDDEN = 4


@dataclass(frozen=True)
class UAVParams:
    mass_kg: float
    max_radius_m: float
    cruise_speed_mps: float
    failure_rate_per_hour: float
    drag_coeff: float = 0.0
    frontal_area_m2: float = 0.1
    initial_vertical_speed_mps: float = 0.0
    max_descent_seconds: float = 300.0


@dataclass(frozen=True)
class HumanParams:
    radius_m: float = 0.3
    height_m: float = 1.7
    impact_energy_threshold_j: float = 290.0
    fatality_steepness: float = 6.0


@dataclass(frozen=True)
class WindSample:
    speed_mps: float
    direction_rad: float


@dataclass(frozen=True)
class RiskThresholds:
    safe: float = 1e-8
    low: float = 1e-7
    medium: float = 1e-6


@dataclass
class RiskAssessmentResult:
    impact_kernel: NumberGrid
    fatality_risk: NumberGrid
    risk_value: NumberGrid
    risk_level: LevelGrid
    flyable: BoolGrid


def zeros(height: int, width: int, value: float = 0.0) -> NumberGrid:
    return [[value for _ in range(width)] for _ in range(height)]


def same_shape(*grids: Sequence[Sequence[object]]) -> Tuple[int, int]:
    if not grids or not grids[0]:
        raise ValueError("at least one non-empty grid is required")
    height = len(grids[0])
    width = len(grids[0][0])
    for grid in grids:
        if len(grid) != height or any(len(row) != width for row in grid):
            raise ValueError("all grids must have the same shape")
    return height, width


def sample_normal(mean: float, std: float) -> float:
    return random.gauss(mean, std) if std > 0 else mean


def simulate_descent_impact(
    uav: UAVParams,
    flight_height_m: float,
    wind: WindSample,
    heading_rad: float = 0.0,
    dt: float = 0.05,
) -> Tuple[float, float, float, float]:
    """
    Returns impact x, y, impact speed and descent angle.

    The model is intentionally compact: thrust is lost, gravity and quadratic
    drag act on the aircraft, and wind translates the horizontal ground track.
    """

    if flight_height_m <= 0:
        raise ValueError("flight_height_m must be positive")
    if uav.mass_kg <= 0:
        raise ValueError("uav mass must be positive")

    rho_air = 1.225
    g = 9.80665
    x = y = 0.0
    z = flight_height_m
    vx = uav.cruise_speed_mps * math.cos(heading_rad)
    vy = uav.cruise_speed_mps * math.sin(heading_rad)
    vz = -abs(uav.initial_vertical_speed_mps)
    wx = wind.speed_mps * math.cos(wind.direction_rad)
    wy = wind.speed_mps * math.sin(wind.direction_rad)
    t = 0.0

    while z > 0 and t < uav.max_descent_seconds:
        rel_vx = vx - wx
        rel_vy = vy - wy
        rel_vz = vz
        rel_speed = math.sqrt(rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz)
        drag_scale = 0.5 * rho_air * uav.drag_coeff * uav.frontal_area_m2 / uav.mass_kg

        ax = -drag_scale * rel_speed * rel_vx
        ay = -drag_scale * rel_speed * rel_vy
        az = -g - drag_scale * rel_speed * rel_vz

        vx += ax * dt
        vy += ay * dt
        vz += az * dt
        x += vx * dt
        y += vy * dt
        z += vz * dt
        t += dt

    horizontal_speed = math.hypot(vx, vy)
    impact_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    descent_angle = math.atan2(abs(vz), max(horizontal_speed, 1e-9))
    return x, y, impact_speed, descent_angle


def build_impact_kernel(
    uav: UAVParams,
    flight_height_m: float,
    cell_size_m: float,
    samples: int = 2000,
    wind_speed_mean_mps: float = 0.0,
    wind_speed_std_mps: float = 0.0,
    wind_dir_mean_rad: float = 0.0,
    wind_dir_std_rad: float = math.pi,
    heading_mean_rad: float = 0.0,
    heading_std_rad: float = 0.0,
    dt: float = 0.05,
) -> Tuple[NumberGrid, float, float]:
    """
    Monte-Carlo impact probability kernel.

    Returns kernel, mean impact speed and mean descent angle. The kernel center
    is the nominal failure point, and each cell stores impact probability.
    """

    if cell_size_m <= 0:
        raise ValueError("cell_size_m must be positive")
    if samples <= 0:
        raise ValueError("samples must be positive")

    impacts = []
    speeds = []
    angles = []
    max_offset = 0

    for _ in range(samples):
        wind = WindSample(
            speed_mps=max(0.0, sample_normal(wind_speed_mean_mps, wind_speed_std_mps)),
            direction_rad=sample_normal(wind_dir_mean_rad, wind_dir_std_rad),
        )
        heading = sample_normal(heading_mean_rad, heading_std_rad)
        x, y, speed, angle = simulate_descent_impact(
            uav, flight_height_m, wind, heading, dt=dt
        )
        ix = int(round(x / cell_size_m))
        iy = int(round(y / cell_size_m))
        impacts.append((ix, iy))
        speeds.append(speed)
        angles.append(angle)
        max_offset = max(max_offset, abs(ix), abs(iy))

    size = max(3, max_offset * 2 + 1)
    if size % 2 == 0:
        size += 1
    center = size // 2
    kernel = zeros(size, size)
    for ix, iy in impacts:
        kx = center + ix
        ky = center + iy
        if 0 <= ky < size and 0 <= kx < size:
            kernel[ky][kx] += 1.0 / samples

    return kernel, sum(speeds) / len(speeds), sum(angles) / len(angles)


def exposure_area_m2(
    uav_radius_m: float,
    human: HumanParams,
    descent_angle_rad: float,
) -> float:
    """
    Capsule-shaped exposed area:
    A_exp = 2 * (r_p + r_uav) * h_p / tan(theta)
            + pi * (r_p + r_uav)^2
    """

    combined_radius = human.radius_m + uav_radius_m
    slide_length = human.height_m / max(math.tan(descent_angle_rad), 1e-9)
    return 2.0 * combined_radius * slide_length + math.pi * combined_radius**2


def collision_probability(pop_density_per_m2: float, exposure_area: float) -> float:
    """Poisson probability that at least one person is inside the exposed area."""

    density = max(0.0, pop_density_per_m2)
    return 1.0 - math.exp(-density * exposure_area)


def fatality_probability(
    impact_energy_j: float,
    shelter_factor: float,
    human: HumanParams,
) -> float:
    """
    Logistic-like fatality model used in many UAV ground-risk studies.

    shelter_factor is in [0, 1]. Larger sheltering reduces the effective impact
    energy before evaluating fatality probability.
    """

    shelter = min(1.0, max(0.0, shelter_factor))
    effective_energy = max(0.0, impact_energy_j * (1.0 - shelter))
    if effective_energy <= 0:
        return 0.0

    ratio = human.impact_energy_threshold_j / effective_energy
    steepness = max(human.fatality_steepness, 1e-9)
    return 1.0 / (1.0 + ratio ** (3.0 / steepness))


def convolved_collision_layer(
    population_density: NumberGrid,
    impact_kernel: NumberGrid,
    exposure_area: float,
) -> NumberGrid:
    height, width = same_shape(population_density)
    kh, kw = same_shape(impact_kernel)
    cy = kh // 2
    cx = kw // 2
    output = zeros(height, width)

    for y in range(height):
        for x in range(width):
            p = 0.0
            for ky in range(kh):
                gy = y + ky - cy
                if gy < 0 or gy >= height:
                    continue
                for kx in range(kw):
                    gx = x + kx - cx
                    if gx < 0 or gx >= width:
                        continue
                    p += impact_kernel[ky][kx] * collision_probability(
                        population_density[gy][gx], exposure_area
                    )
            output[y][x] = min(1.0, max(0.0, p))
    return output


def classify_risk(value: float, thresholds: RiskThresholds = RiskThresholds()) -> RiskLevel:
    if value >= thresholds.medium:
        return RiskLevel.HIGH
    if value >= thresholds.low:
        return RiskLevel.MEDIUM
    if value >= thresholds.safe:
        return RiskLevel.LOW
    return RiskLevel.SAFE


def assess_ground_risk(
    population_density: NumberGrid,
    shelter_factor: NumberGrid,
    impact_kernel: NumberGrid,
    uav: UAVParams,
    human: HumanParams,
    impact_speed_mps: float,
    descent_angle_rad: float,
    no_fly: Optional[BoolGrid] = None,
    obstacle_height_m: Optional[NumberGrid] = None,
    flight_height_m: Optional[float] = None,
    special_area_level: Optional[LevelGrid] = None,
    thresholds: RiskThresholds = RiskThresholds(),
) -> RiskAssessmentResult:
    height, width = same_shape(population_density, shelter_factor)
    if no_fly is not None:
        same_shape(population_density, no_fly)
    if obstacle_height_m is not None:
        same_shape(population_density, obstacle_height_m)
    if special_area_level is not None:
        same_shape(population_density, special_area_level)

    exposure = exposure_area_m2(uav.max_radius_m, human, descent_angle_rad)
    impact_energy = 0.5 * uav.mass_kg * impact_speed_mps**2
    collision = convolved_collision_layer(population_density, impact_kernel, exposure)

    fatality_risk = zeros(height, width)
    risk_value = zeros(height, width)
    risk_level: LevelGrid = [[RiskLevel.SAFE for _ in range(width)] for _ in range(height)]
    flyable: BoolGrid = [[True for _ in range(width)] for _ in range(height)]

    for y in range(height):
        for x in range(width):
            blocked = False
            if no_fly is not None and no_fly[y][x]:
                blocked = True
            if (
                obstacle_height_m is not None
                and flight_height_m is not None
                and obstacle_height_m[y][x] >= flight_height_m
            ):
                blocked = True

            pf = fatality_probability(impact_energy, shelter_factor[y][x], human)
            value = uav.failure_rate_per_hour * collision[y][x] * pf
            base_level = classify_risk(value, thresholds)
            if special_area_level is not None:
                base_level = max(base_level, RiskLevel(special_area_level[y][x]))
            if blocked:
                base_level = RiskLevel.FORBIDDEN

            fatality_risk[y][x] = pf
            risk_value[y][x] = value
            risk_level[y][x] = int(base_level)
            flyable[y][x] = base_level < RiskLevel.HIGH

    return RiskAssessmentResult(
        impact_kernel=impact_kernel,
        fatality_risk=fatality_risk,
        risk_value=risk_value,
        risk_level=risk_level,
        flyable=flyable,
    )


def safe_airspace_mask(
    result: RiskAssessmentResult,
    max_allowed_level: RiskLevel = RiskLevel.LOW,
) -> BoolGrid:
    return [
        [
            result.flyable[y][x] and result.risk_level[y][x] <= max_allowed_level
            for x in range(len(result.risk_level[0]))
        ]
        for y in range(len(result.risk_level))
    ]


def path_planning_cost(
    result: RiskAssessmentResult,
    forbidden_cost: float = math.inf,
    risk_weight: float = 1e8,
    level_weight: float = 10.0,
) -> NumberGrid:
    height, width = same_shape(result.risk_value)
    cost = zeros(height, width)
    for y in range(height):
        for x in range(width):
            if result.risk_level[y][x] >= RiskLevel.FORBIDDEN:
                cost[y][x] = forbidden_cost
            else:
                cost[y][x] = 1.0 + risk_weight * result.risk_value[y][x] + level_weight * result.risk_level[y][x]
    return cost


def cells_from_polygons_placeholder(items: Iterable[Tuple[int, int]], height: int, width: int) -> BoolGrid:
    """
    Convenience helper for early experiments when GIS rasterization is not wired
    in yet. Pass already-rasterized (x, y) cells and get a boolean layer.
    """

    grid = [[False for _ in range(width)] for _ in range(height)]
    for x, y in items:
        if 0 <= y < height and 0 <= x < width:
            grid[y][x] = True
    return grid
