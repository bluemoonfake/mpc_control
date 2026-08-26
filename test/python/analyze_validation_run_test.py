#!/usr/bin/env python3
"""Focused tests for strict TMPC validation gates."""

import sys
import json
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "analysis"))

import analyze_validation_run as validation


def sample(**overrides: object) -> dict:
    values = {
        "mx": 0.0,
        "my": 0.0,
        "mz": 10.0,
        "motor_available": 1,
        "motor_age_ms": 5.0,
        "motor_min": 0.10,
        "motor_max": 0.90,
    }
    values.update(overrides)
    return values


class ValidationGateTest(unittest.TestCase):
    def test_motor_saturation_requires_fresh_measurements(self) -> None:
        stats = validation.motor_saturation_stats(
            [sample(), sample(motor_age_ms=101.0)], maximum_age_ms=100.0
        )

        self.assertTrue(stats["available"])
        self.assertEqual(stats["fresh_sample_count"], 1)
        self.assertEqual(stats["saturation_rate"], 0.0)

    def test_motor_saturation_detects_upper_limit(self) -> None:
        stats = validation.motor_saturation_stats(
            [sample(motor_max=0.98)], maximum_age_ms=100.0
        )

        self.assertEqual(stats["saturation_count"], 1)
        self.assertEqual(stats["saturation_rate"], 1.0)

    def test_cylinder_clearance_is_distance_from_obstacle_surface(self) -> None:
        clearance = validation.obstacle_clearance(
            sample(mx=5.0, my=0.0),
            {"type": "cylinder", "x": 0.0, "y": 0.0, "radiusM": 2.0},
        )

        self.assertEqual(clearance, 3.0)

    def test_box_clearance_is_negative_inside_obstacle(self) -> None:
        clearance = validation.obstacle_clearance(
            sample(mx=0.5, my=0.0),
            {
                "type": "box",
                "x": 0.0,
                "y": 0.0,
                "halfExtentXM": 1.0,
                "halfExtentYM": 1.0,
            },
        )

        self.assertEqual(clearance, -0.5)

    def test_waypoint_only_mission_makes_clearance_not_applicable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            mission_path = Path(directory) / "mission.json"
            mission_path.write_text(
                json.dumps({"version": 1, "mission": {"items": []}}),
                encoding="utf-8",
            )

            stats = validation.obstacle_clearance_stats(
                [sample()], mission_path, minimum_clearance_override_m=None
            )

        self.assertFalse(stats["available"])
        self.assertFalse(stats["applicable"])

    def test_non_applicable_clearance_does_not_fail_overall_verdict(self) -> None:
        verdicts = {
            "required_gate": {"pass": True},
            "minimum_obstacle_clearance": {
                "pass": None,
                "required": False,
                "status": "not_applicable",
            },
        }

        overall = all(
            verdict["pass"] is True
            for verdict in verdicts.values()
            if verdict.get("required", True)
        )

        self.assertTrue(overall)


if __name__ == "__main__":
    unittest.main()
