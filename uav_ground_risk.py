"""
Grid-based ground-risk assessment for urban low-altitude UAV operations.

The implementation follows the paper's main workflow:
1. simulate a UAV failure descent and build an impact-position probability kernel;
2. combine the kernel with population density, exposure area and sheltering effect;
3. fuse no-fly, obstacle and special-area layers into a final risk map.

All grids are row-major: grid[y][x].
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from enum import IntEnum
import html
import math
import random
from typing import Callable, Iterable, List, Optional, Sequence, Tuple, TypeVar


NumberGrid = List[List[float]]
BoolGrid = List[List[bool]]
LevelGrid = List[List[int]]
T = TypeVar("T")


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


def _escape(text: object) -> str:
    return html.escape(str(text), quote=True)


def read_csv_grid(path: str, parser: Callable[[str], T]) -> List[List[T]]:
    with open(path, newline="", encoding="utf-8-sig") as handle:
        rows = [
            [parser(cell.strip()) for cell in row]
            for row in csv.reader(handle)
            if row and any(cell.strip() for cell in row)
        ]
    same_shape(rows)
    return rows


def read_number_grid_csv(path: str) -> NumberGrid:
    return read_csv_grid(path, float)


def parse_bool_cell(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", ""}:
        return False
    raise ValueError(f"invalid boolean cell value: {value!r}")


def read_bool_grid_csv(path: str) -> BoolGrid:
    return read_csv_grid(path, parse_bool_cell)


def read_level_grid_csv(path: str) -> LevelGrid:
    return read_csv_grid(path, lambda value: int(RiskLevel(int(value))))


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


def _hex_to_rgb(color: str) -> Tuple[int, int, int]:
    color = color.lstrip("#")
    if len(color) != 6:
        raise ValueError("color must be in #RRGGBB format")
    return int(color[0:2], 16), int(color[2:4], 16), int(color[4:6], 16)


def _rgb_to_hex(rgb: Tuple[int, int, int]) -> str:
    return "#{:02x}{:02x}{:02x}".format(*rgb)


def _mix_color(start: str, end: str, ratio: float) -> str:
    ratio = min(1.0, max(0.0, ratio))
    a = _hex_to_rgb(start)
    b = _hex_to_rgb(end)
    return _rgb_to_hex(tuple(round(a[i] + (b[i] - a[i]) * ratio) for i in range(3)))


def _finite_values(grid: NumberGrid) -> List[float]:
    return [value for row in grid for value in row if math.isfinite(value)]


def _linear_color(value: float, minimum: float, maximum: float, start: str, end: str) -> str:
    if not math.isfinite(value):
        return "#111827"
    if maximum <= minimum:
        return end
    return _mix_color(start, end, (value - minimum) / (maximum - minimum))


def _log_color(value: float, positive_min: float, positive_max: float, start: str, end: str) -> str:
    if not math.isfinite(value):
        return "#111827"
    if value <= 0:
        return start
    if positive_max <= positive_min:
        return end
    ratio = (math.log10(value) - math.log10(positive_min)) / (
        math.log10(positive_max) - math.log10(positive_min)
    )
    return _mix_color(start, end, ratio)


def render_number_grid_svg(
    grid: NumberGrid,
    title: str,
    cell_size: int = 28,
    start_color: str = "#f8fafc",
    end_color: str = "#dc2626",
    log_scale: bool = False,
    value_format: str = ".1e",
) -> str:
    height, width = same_shape(grid)
    finite = _finite_values(grid)
    minimum = min(finite) if finite else 0.0
    maximum = max(finite) if finite else 1.0
    positive = [value for value in finite if value > 0]
    positive_min = min(positive) if positive else 1.0
    positive_max = max(positive) if positive else 1.0
    padding_top = 44
    padding_left = 44
    svg_width = padding_left + width * cell_size + 16
    svg_height = padding_top + height * cell_size + 48
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_width}" height="{svg_height}" viewBox="0 0 {svg_width} {svg_height}" role="img" aria-label="{_escape(title)}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{padding_left}" y="24" font-family="Arial, sans-serif" font-size="16" font-weight="700" fill="#111827">{_escape(title)}</text>',
    ]

    for y, row in enumerate(grid):
        parts.append(
            f'<text x="{padding_left - 10}" y="{padding_top + y * cell_size + cell_size * 0.65:.1f}" text-anchor="end" font-family="Arial, sans-serif" font-size="10" fill="#475569">{y}</text>'
        )
        for x, value in enumerate(row):
            if y == 0:
                parts.append(
                    f'<text x="{padding_left + x * cell_size + cell_size / 2:.1f}" y="{padding_top - 8}" text-anchor="middle" font-family="Arial, sans-serif" font-size="10" fill="#475569">{x}</text>'
                )
            color = (
                _log_color(value, positive_min, positive_max, start_color, end_color)
                if log_scale
                else _linear_color(value, minimum, maximum, start_color, end_color)
            )
            label = "inf" if not math.isfinite(value) else format(value, value_format)
            parts.extend(
                [
                    f'<rect x="{padding_left + x * cell_size}" y="{padding_top + y * cell_size}" width="{cell_size}" height="{cell_size}" fill="{color}" stroke="#ffffff" stroke-width="1"/>',
                    f'<title>({x}, {y}) {label}</title>',
                ]
            )

    parts.extend(
        [
            f'<text x="{padding_left}" y="{svg_height - 18}" font-family="Arial, sans-serif" font-size="11" fill="#475569">min: {format(minimum, value_format)}  max: {format(maximum, value_format)}</text>',
            "</svg>",
        ]
    )
    return "\n".join(parts)


def render_level_grid_svg(
    grid: LevelGrid,
    title: str,
    cell_size: int = 28,
) -> str:
    height, width = same_shape(grid)
    colors = {
        RiskLevel.SAFE: "#d1fae5",
        RiskLevel.LOW: "#fde68a",
        RiskLevel.MEDIUM: "#fb923c",
        RiskLevel.HIGH: "#ef4444",
        RiskLevel.FORBIDDEN: "#111827",
    }
    labels = {
        RiskLevel.SAFE: "SAFE",
        RiskLevel.LOW: "LOW",
        RiskLevel.MEDIUM: "MED",
        RiskLevel.HIGH: "HIGH",
        RiskLevel.FORBIDDEN: "FORB",
    }
    padding_top = 44
    padding_left = 44
    legend_height = 34
    svg_width = padding_left + width * cell_size + 16
    svg_height = padding_top + height * cell_size + legend_height + 30
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_width}" height="{svg_height}" viewBox="0 0 {svg_width} {svg_height}" role="img" aria-label="{_escape(title)}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{padding_left}" y="24" font-family="Arial, sans-serif" font-size="16" font-weight="700" fill="#111827">{_escape(title)}</text>',
    ]

    for y, row in enumerate(grid):
        parts.append(
            f'<text x="{padding_left - 10}" y="{padding_top + y * cell_size + cell_size * 0.65:.1f}" text-anchor="end" font-family="Arial, sans-serif" font-size="10" fill="#475569">{y}</text>'
        )
        for x, value in enumerate(row):
            level = RiskLevel(value)
            if y == 0:
                parts.append(
                    f'<text x="{padding_left + x * cell_size + cell_size / 2:.1f}" y="{padding_top - 8}" text-anchor="middle" font-family="Arial, sans-serif" font-size="10" fill="#475569">{x}</text>'
                )
            text_color = "#ffffff" if level in {RiskLevel.HIGH, RiskLevel.FORBIDDEN} else "#111827"
            parts.extend(
                [
                    f'<rect x="{padding_left + x * cell_size}" y="{padding_top + y * cell_size}" width="{cell_size}" height="{cell_size}" fill="{colors[level]}" stroke="#ffffff" stroke-width="1"/>',
                    f'<text x="{padding_left + x * cell_size + cell_size / 2:.1f}" y="{padding_top + y * cell_size + cell_size * 0.64:.1f}" text-anchor="middle" font-family="Arial, sans-serif" font-size="8" font-weight="700" fill="{text_color}">{labels[level]}</text>',
                    f'<title>({x}, {y}) {labels[level]}</title>',
                ]
            )

    legend_y = padding_top + height * cell_size + 18
    legend_x = padding_left
    for level in RiskLevel:
        parts.extend(
            [
                f'<rect x="{legend_x}" y="{legend_y}" width="12" height="12" fill="{colors[level]}" stroke="#cbd5e1" stroke-width="0.5"/>',
                f'<text x="{legend_x + 16}" y="{legend_y + 10}" font-family="Arial, sans-serif" font-size="10" fill="#334155">{labels[level]}</text>',
            ]
        )
        legend_x += 58

    parts.append("</svg>")
    return "\n".join(parts)


def render_bool_grid_svg(
    grid: BoolGrid,
    title: str,
    cell_size: int = 28,
    true_label: str = "flyable",
    false_label: str = "blocked",
) -> str:
    numeric = [[1.0 if value else 0.0 for value in row] for row in grid]
    svg = render_number_grid_svg(
        numeric,
        title=title,
        cell_size=cell_size,
        start_color="#ef4444",
        end_color="#22c55e",
        log_scale=False,
        value_format=".0f",
    )
    return svg.replace("min: 0  max: 1", f"{_escape(false_label)}: 0  {_escape(true_label)}: 1")


def render_result_visualization_html(
    result: RiskAssessmentResult,
    title: str = "UAV Ground Risk Visualization",
    cell_size: int = 28,
) -> str:
    safe = safe_airspace_mask(result)
    cost = path_planning_cost(result)
    panels = [
        render_number_grid_svg(
            result.risk_value,
            "Fatality Risk R(x, y)",
            cell_size=cell_size,
            start_color="#eff6ff",
            end_color="#b91c1c",
            log_scale=True,
        ),
        render_level_grid_svg(result.risk_level, "Fused Risk Level", cell_size=cell_size),
        render_bool_grid_svg(safe, "Safe Airspace Mask", cell_size=cell_size),
        render_number_grid_svg(
            cost,
            "Path Planning Cost",
            cell_size=cell_size,
            start_color="#f8fafc",
            end_color="#7c2d12",
            log_scale=False,
            value_format=".2f",
        ),
    ]
    return "\n".join(
        [
            "<!doctype html>",
            '<html lang="zh-CN">',
            "<head>",
            '<meta charset="utf-8"/>',
            f"<title>{_escape(title)}</title>",
            "<style>",
            "body{margin:24px;font-family:Arial,'Microsoft YaHei',sans-serif;background:#f8fafc;color:#111827;}",
            "h1{font-size:22px;margin:0 0 18px;}",
            ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px;align-items:start;}",
            ".panel{background:#fff;border:1px solid #e5e7eb;border-radius:8px;padding:12px;box-shadow:0 1px 2px rgba(15,23,42,.05);overflow:auto;}",
            "svg{max-width:100%;height:auto;display:block;}",
            "</style>",
            "</head>",
            "<body>",
            f"<h1>{_escape(title)}</h1>",
            '<div class="grid">',
            *[f'<section class="panel">{panel}</section>' for panel in panels],
            "</div>",
            "</body>",
            "</html>",
        ]
    )


def write_result_visualization_html(
    result: RiskAssessmentResult,
    output_path: str,
    title: str = "UAV Ground Risk Visualization",
    cell_size: int = 28,
) -> None:
    html_text = render_result_visualization_html(result, title=title, cell_size=cell_size)
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write(html_text)


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
