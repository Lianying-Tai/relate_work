import math
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from uav_ground_risk import (
    HumanParams,
    RiskLevel,
    UAVParams,
    assess_ground_risk,
    build_impact_kernel,
    path_planning_cost,
    read_bool_grid_csv,
    read_level_grid_csv,
    read_number_grid_csv,
    render_level_grid_svg,
    render_result_visualization_html,
    safe_airspace_mask,
    write_result_visualization_html,
)


class GroundRiskAssessmentTests(unittest.TestCase):
    def setUp(self):
        self.uav = UAVParams(
            mass_kg=1.4,
            max_radius_m=0.3,
            cruise_speed_mps=16.0,
            failure_rate_per_hour=1e-5,
            drag_coeff=0.6,
            frontal_area_m2=0.08,
        )
        self.human = HumanParams()

    def test_impact_kernel_is_normalized(self):
        random.seed(7)

        kernel, impact_speed, descent_angle = build_impact_kernel(
            uav=self.uav,
            flight_height_m=100.0,
            cell_size_m=100.0,
            samples=50,
            wind_speed_mean_mps=3.0,
            wind_speed_std_mps=1.0,
        )

        self.assertAlmostEqual(sum(sum(row) for row in kernel), 1.0)
        self.assertGreater(impact_speed, 0.0)
        self.assertGreater(descent_angle, 0.0)

    def test_risk_layers_mark_blocked_and_special_cells(self):
        height = width = 8
        population = [[0.002 for _ in range(width)] for _ in range(height)]
        shelter = [[0.27 for _ in range(width)] for _ in range(height)]
        no_fly = [[False for _ in range(width)] for _ in range(height)]
        obstacles = [[0.0 for _ in range(width)] for _ in range(height)]
        special = [[RiskLevel.SAFE for _ in range(width)] for _ in range(height)]

        no_fly[2][3] = True
        obstacles[4][4] = 120.0
        special[5][1] = RiskLevel.HIGH

        result = assess_ground_risk(
            population_density=population,
            shelter_factor=shelter,
            impact_kernel=[[1.0]],
            uav=self.uav,
            human=self.human,
            impact_speed_mps=30.0,
            descent_angle_rad=math.radians(60.0),
            no_fly=no_fly,
            obstacle_height_m=obstacles,
            flight_height_m=100.0,
            special_area_level=special,
        )

        self.assertEqual(result.risk_level[2][3], RiskLevel.FORBIDDEN)
        self.assertEqual(result.risk_level[4][4], RiskLevel.FORBIDDEN)
        self.assertEqual(result.risk_level[5][1], RiskLevel.HIGH)
        self.assertFalse(result.flyable[2][3])
        self.assertFalse(result.flyable[5][1])

    def test_planning_helpers_convert_risk_to_mask_and_cost(self):
        population = [[0.0, 0.002], [0.002, 0.002]]
        shelter = [[1.0, 0.27], [0.27, 0.27]]
        no_fly = [[False, True], [False, False]]

        result = assess_ground_risk(
            population_density=population,
            shelter_factor=shelter,
            impact_kernel=[[1.0]],
            uav=self.uav,
            human=self.human,
            impact_speed_mps=30.0,
            descent_angle_rad=math.radians(60.0),
            no_fly=no_fly,
        )

        safe = safe_airspace_mask(result, max_allowed_level=RiskLevel.LOW)
        cost = path_planning_cost(result)

        self.assertTrue(safe[0][0])
        self.assertFalse(safe[0][1])
        self.assertEqual(cost[0][1], math.inf)
        self.assertGreaterEqual(cost[1][0], 1.0)

    def test_csv_grid_readers_parse_common_layer_types(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            number_path = os.path.join(tmpdir, "population.csv")
            bool_path = os.path.join(tmpdir, "no_fly.csv")
            level_path = os.path.join(tmpdir, "special.csv")

            with open(number_path, "w", encoding="utf-8") as handle:
                handle.write("0.1,0.2\n0.3,0.4\n")
            with open(bool_path, "w", encoding="utf-8") as handle:
                handle.write("0,1\nfalse,true\n")
            with open(level_path, "w", encoding="utf-8") as handle:
                handle.write("0,1\n2,3\n")

            self.assertEqual(read_number_grid_csv(number_path), [[0.1, 0.2], [0.3, 0.4]])
            self.assertEqual(read_bool_grid_csv(bool_path), [[False, True], [False, True]])
            self.assertEqual(read_level_grid_csv(level_path), [[0, 1], [2, 3]])

    def test_visualization_helpers_render_svg_and_html(self):
        result = assess_ground_risk(
            population_density=[[0.0, 0.002], [0.002, 0.002]],
            shelter_factor=[[1.0, 0.27], [0.27, 0.27]],
            impact_kernel=[[1.0]],
            uav=self.uav,
            human=self.human,
            impact_speed_mps=30.0,
            descent_angle_rad=math.radians(60.0),
            no_fly=[[False, True], [False, False]],
        )

        svg = render_level_grid_svg(result.risk_level, "Risk Level")
        html = render_result_visualization_html(result, title="Example Risk Map")

        self.assertIn("<svg", svg)
        self.assertIn("FORB", svg)
        self.assertIn("Fatality Risk", html)
        self.assertIn("Path Planning Cost", html)

        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = os.path.join(tmpdir, "risk.html")
            write_result_visualization_html(result, output_path, title="Example Risk Map")
            with open(output_path, encoding="utf-8") as handle:
                saved = handle.read()

        self.assertIn("<!doctype html>", saved)
        self.assertIn("Example Risk Map", saved)


if __name__ == "__main__":
    unittest.main()
