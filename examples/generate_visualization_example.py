import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from uav_ground_risk import (
    HumanParams,
    RiskLevel,
    UAVParams,
    assess_ground_risk,
    build_impact_kernel,
    write_result_visualization_html,
)


def main() -> None:
    random.seed(3)
    height = width = 12

    population = [
        [0.0004 + 0.00025 * x + 0.00012 * y for x in range(width)]
        for y in range(height)
    ]
    shelter = [
        [0.15 + 0.03 * ((x + y) % 5) for x in range(width)]
        for y in range(height)
    ]
    no_fly = [[False for _ in range(width)] for _ in range(height)]
    obstacles = [[0.0 for _ in range(width)] for _ in range(height)]
    special = [[RiskLevel.SAFE for _ in range(width)] for _ in range(height)]

    for y in range(2, 5):
        no_fly[y][8] = True
    for x in range(3, 7):
        obstacles[9][x] = 130.0
    for y in range(8, 11):
        special[y][1] = RiskLevel.HIGH

    uav = UAVParams(
        mass_kg=1.4,
        max_radius_m=0.3,
        cruise_speed_mps=16.0,
        failure_rate_per_hour=1e-5,
        drag_coeff=0.6,
        frontal_area_m2=0.08,
    )
    human = HumanParams()

    kernel, impact_speed, descent_angle = build_impact_kernel(
        uav=uav,
        flight_height_m=100.0,
        cell_size_m=100.0,
        samples=200,
        wind_speed_mean_mps=3.0,
        wind_speed_std_mps=1.0,
    )

    result = assess_ground_risk(
        population_density=population,
        shelter_factor=shelter,
        impact_kernel=kernel,
        uav=uav,
        human=human,
        impact_speed_mps=impact_speed,
        descent_angle_rad=descent_angle,
        no_fly=no_fly,
        obstacle_height_m=obstacles,
        flight_height_m=100.0,
        special_area_level=special,
    )

    output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "outputs")
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "risk_visualization.html")
    write_result_visualization_html(
        result,
        output_path,
        title="Urban Low-Altitude UAV Ground Risk Visualization",
        cell_size=30,
    )
    print(output_path)


if __name__ == "__main__":
    main()
