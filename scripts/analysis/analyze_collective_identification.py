#!/usr/bin/env python3
"""Fit and qualify a first-order collective-force model from SITL telemetry."""

import argparse
import csv
import json
import math
from pathlib import Path


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def median(values):
    ordered = sorted(values)
    count = len(ordered)
    if count == 0:
        return float("nan")
    middle = count // 2
    return ordered[middle] if count % 2 else 0.5 * (ordered[middle - 1] + ordered[middle])


def standard_deviation(values):
    if len(values) < 2:
        return 0.0
    average = mean(values)
    return math.sqrt(mean([(value - average) ** 2 for value in values]))


def parse_float(row, key, fallback=float("nan")):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return fallback
    return value if math.isfinite(value) else fallback


def load_rows(path):
    with path.open(newline="") as stream:
        source_rows = list(csv.DictReader(stream))
    rows = []
    for source in source_rows:
        time_seconds = parse_float(source, "time_s")
        command = parse_float(source, "command_collective_m_s2")
        applied = parse_float(source, "applied_collective_m_s2", command)
        measured = parse_float(source, "measured_collective_m_s2")
        filtered = parse_float(source, "filtered_collective_m_s2", measured)
        if not all(math.isfinite(value) for value in (time_seconds, command, applied, measured, filtered)):
            continue
        rows.append(
            {
                "time_s": time_seconds,
                "phase": source.get("phase", "unknown"),
                "command_collective_m_s2": command,
                "applied_collective_m_s2": applied,
                "measured_collective_m_s2": measured,
                "filtered_collective_m_s2": filtered,
                "hover_thrust": parse_float(source, "hover_thrust"),
                "tilt_rad": parse_float(source, "tilt_rad"),
                "state_command_skew_ms": parse_float(source, "state_command_skew_ms"),
                "raw_vertical_acceleration_m_s2": parse_float(
                    source, "raw_vertical_acceleration_m_s2"),
                "motor_mean": parse_float(source, "motor_mean"),
            }
        )
    if len(rows) < 20:
        raise ValueError("identification CSV contains fewer than 20 valid samples")
    return rows


def select_excitation_rows(rows, transition_guard_seconds, maximum_tilt_rad, maximum_skew_ms):
    selected_indices = []
    rejected = {"settle": 0, "transition": 0, "tilt": 0, "timestamp_skew": 0}
    current_phase = None
    phase_started_at = None
    for index, row in enumerate(rows):
        phase = row["phase"]
        if phase != current_phase:
            current_phase = phase
            phase_started_at = row["time_s"]
        if phase not in ("high", "low"):
            rejected["settle"] += 1
            continue
        if row["time_s"] - phase_started_at < transition_guard_seconds:
            rejected["transition"] += 1
            continue
        if math.isfinite(row["tilt_rad"]) and row["tilt_rad"] > maximum_tilt_rad:
            rejected["tilt"] += 1
            continue
        if math.isfinite(row["state_command_skew_ms"]) and row["state_command_skew_ms"] > maximum_skew_ms:
            rejected["timestamp_skew"] += 1
            continue
        selected_indices.append(index)
    return selected_indices, rejected


def sample_period(rows):
    periods = [
        rows[index]["time_s"] - rows[index - 1]["time_s"]
        for index in range(1, len(rows))
        if rows[index]["time_s"] > rows[index - 1]["time_s"]
    ]
    if not periods:
        raise ValueError("identification timestamps are not strictly increasing")
    return max(median(periods), 1.0e-4)


def simulate_response(rows, tau_seconds, delay_samples, input_center):
    response = [0.0] * len(rows)
    for index in range(1, len(rows)):
        dt = max(rows[index]["time_s"] - rows[index - 1]["time_s"], 1.0e-4)
        alpha = math.exp(-dt / tau_seconds)
        source_index = max(0, index - delay_samples)
        command_delta = rows[source_index]["applied_collective_m_s2"] - input_center
        response[index] = alpha * response[index - 1] + (1.0 - alpha) * command_delta
    return response


def weighted_affine_fit(inputs, outputs, weights):
    total_weight = sum(weights)
    if total_weight <= 1.0e-12:
        return float("nan"), float("nan")
    mean_input = sum(weight * value for weight, value in zip(weights, inputs)) / total_weight
    mean_output = sum(weight * value for weight, value in zip(weights, outputs)) / total_weight
    denominator = sum(
        weight * (value - mean_input) ** 2 for weight, value in zip(weights, inputs))
    if denominator <= 1.0e-12:
        return float("nan"), float("nan")
    gain = sum(
        weight * (input_value - mean_input) * (output_value - mean_output)
        for weight, input_value, output_value in zip(weights, inputs, outputs)
    ) / denominator
    return mean_output - gain * mean_input, gain


def robust_affine_fit(inputs, outputs):
    weights = [1.0] * len(inputs)
    bias, gain = weighted_affine_fit(inputs, outputs, weights)
    if not math.isfinite(gain):
        return float("nan"), float("nan"), [], float("nan")
    for _ in range(8):
        residuals = [actual - (bias + gain * value) for value, actual in zip(inputs, outputs)]
        residual_center = median(residuals)
        robust_scale = max(
            1.4826 * median([abs(value - residual_center) for value in residuals]), 1.0e-6)
        huber_limit = 1.5 * robust_scale
        weights = [
            1.0 if abs(value - residual_center) <= huber_limit
            else huber_limit / abs(value - residual_center)
            for value in residuals
        ]
        next_bias, next_gain = weighted_affine_fit(inputs, outputs, weights)
        if not math.isfinite(next_gain):
            break
        if abs(next_bias - bias) < 1.0e-8 and abs(next_gain - gain) < 1.0e-8:
            bias, gain = next_bias, next_gain
            break
        bias, gain = next_bias, next_gain
    residuals = [actual - (bias + gain * value) for value, actual in zip(inputs, outputs)]
    residual_center = median(residuals)
    robust_scale = max(
        1.4826 * median([abs(value - residual_center) for value in residuals]), 1.0e-6)
    return bias, gain, residuals, robust_scale


def fit_model(source_rows, selected_indices, minimum_tau_seconds, maximum_tau_seconds,
              maximum_delay_seconds):
    selected_rows = [source_rows[index] for index in selected_indices]
    input_center = mean([row["applied_collective_m_s2"] for row in selected_rows])
    outputs = [row["filtered_collective_m_s2"] for row in selected_rows]
    period = sample_period(source_rows)
    maximum_delay_samples = max(0, int(maximum_delay_seconds / period))
    best = None
    tau_seconds = minimum_tau_seconds
    while tau_seconds <= maximum_tau_seconds * (1.0 + 1.0e-12):
        for delay_samples in range(maximum_delay_samples + 1):
            complete_response = simulate_response(
                source_rows, tau_seconds, delay_samples, input_center)
            response = [complete_response[index] for index in selected_indices]
            bias, gain, residuals, robust_scale = robust_affine_fit(response, outputs)
            if not math.isfinite(gain):
                continue
            rmse = math.sqrt(mean([value * value for value in residuals]))
            candidate = {
                "fit_rmse_m_s2": rmse,
                "collective_time_constant_seconds": tau_seconds,
                "collective_delay_seconds": delay_samples * period,
                "delay_samples": delay_samples,
                "fit_bias_m_s2": bias,
                "collective_gain": gain,
                "robust_residual_scale_m_s2": robust_scale,
                "response": response,
                "residuals": residuals,
            }
            if best is None or candidate["fit_rmse_m_s2"] < best["fit_rmse_m_s2"]:
                best = candidate
        tau_seconds *= 1.08
    if best is None:
        raise ValueError("could not fit a first-order collective model")
    best["sample_period_seconds"] = period
    best["input_center_m_s2"] = input_center
    return best


def quality_report(rows, fit, minimum_tau_seconds, maximum_tau_seconds):
    input_values = [row["applied_collective_m_s2"] for row in rows]
    output_values = [row["filtered_collective_m_s2"] for row in rows]
    input_standard_deviation = standard_deviation(input_values)
    output_standard_deviation = standard_deviation(output_values)
    explained_standard_deviation = abs(fit["collective_gain"]) * standard_deviation(
        fit["response"])
    snr = explained_standard_deviation / max(fit["robust_residual_scale_m_s2"], 1.0e-6)
    output_mean = mean(output_values)
    total_squared_error = sum((value - output_mean) ** 2 for value in output_values)
    residual_squared_error = sum(value * value for value in fit["residuals"])
    r_squared = 1.0 - residual_squared_error / total_squared_error if total_squared_error > 1.0e-12 else float("nan")
    at_tau_lower_bound = fit["collective_time_constant_seconds"] <= minimum_tau_seconds * 1.08
    at_tau_upper_bound = fit["collective_time_constant_seconds"] >= maximum_tau_seconds / 1.08
    rejection_reasons = []
    if len(rows) < 120:
        rejection_reasons.append("too_few_samples")
    if input_standard_deviation < 0.08:
        rejection_reasons.append("insufficient_input_excitation")
    if not math.isfinite(r_squared) or r_squared < 0.55:
        rejection_reasons.append("low_explained_variance")
    if snr < 2.5:
        rejection_reasons.append("low_signal_to_noise_ratio")
    if fit["fit_rmse_m_s2"] > 0.12:
        rejection_reasons.append("fit_rmse_too_high")
    if fit["collective_gain"] < 0.7 or fit["collective_gain"] > 1.3:
        rejection_reasons.append("collective_gain_not_unit_consistent")
    if at_tau_lower_bound or at_tau_upper_bound:
        rejection_reasons.append("time_constant_on_search_boundary")
    confidence = max(0.0, min(1.0,
        0.35 * max(0.0, min(1.0, r_squared if math.isfinite(r_squared) else 0.0)) +
        0.35 * max(0.0, min(1.0, snr / 5.0)) +
        0.30 * max(0.0, min(1.0, input_standard_deviation / 0.20))))
    return {
        "accepted": not rejection_reasons,
        "rejection_reasons": rejection_reasons,
        "confidence": confidence,
        "input_standard_deviation_m_s2": input_standard_deviation,
        "output_standard_deviation_m_s2": output_standard_deviation,
        "explained_standard_deviation_m_s2": explained_standard_deviation,
        "signal_to_noise_ratio": snr,
        "r_squared": r_squared,
        "time_constant_at_lower_bound": at_tau_lower_bound,
        "time_constant_at_upper_bound": at_tau_upper_bound,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--transition-guard-seconds", type=float, default=0.08)
    parser.add_argument("--maximum-tilt-rad", type=float, default=0.08)
    parser.add_argument("--maximum-state-command-skew-ms", type=float, default=80.0)
    parser.add_argument("--minimum-time-constant-seconds", type=float, default=0.01)
    parser.add_argument("--maximum-time-constant-seconds", type=float, default=1.50)
    parser.add_argument("--maximum-delay-seconds", type=float, default=0.50)
    parser.add_argument("--require-accepted", action="store_true")
    args = parser.parse_args()
    if (args.transition_guard_seconds < 0.0 or args.maximum_tilt_rad <= 0.0 or
            args.maximum_state_command_skew_ms <= 0.0 or
            args.minimum_time_constant_seconds <= 0.0 or
            args.maximum_time_constant_seconds <= args.minimum_time_constant_seconds or
            args.maximum_delay_seconds < 0.0):
        raise ValueError("invalid analysis limits")

    source_rows = load_rows(args.input)
    selected_indices, rejected = select_excitation_rows(
        source_rows, args.transition_guard_seconds, args.maximum_tilt_rad,
        args.maximum_state_command_skew_ms)
    if len(selected_indices) < 20:
        raise ValueError("fewer than 20 samples remain after quality filtering")
    rows = [source_rows[index] for index in selected_indices]
    fit = fit_model(
        source_rows, selected_indices, args.minimum_time_constant_seconds,
        args.maximum_time_constant_seconds, args.maximum_delay_seconds)
    quality = quality_report(
        rows, fit, args.minimum_time_constant_seconds,
        args.maximum_time_constant_seconds)
    result = {
        "source_samples": len(source_rows),
        "selected_samples": len(rows),
        "discarded_samples": rejected,
        "duration_seconds": rows[-1]["time_s"] - rows[0]["time_s"],
        "identification_input": "applied_collective_m_s2",
        "identification_output": "filtered_collective_m_s2",
        "hover_collective_specific_force_m_s2": mean(
            [row["filtered_collective_m_s2"] for row in rows]),
        "hover_thrust_mean": mean([
            row["hover_thrust"] for row in rows if math.isfinite(row["hover_thrust"])]),
        "applied_min_m_s2": min(row["applied_collective_m_s2"] for row in rows),
        "applied_max_m_s2": max(row["applied_collective_m_s2"] for row in rows),
        "measured_min_m_s2": min(row["filtered_collective_m_s2"] for row in rows),
        "measured_max_m_s2": max(row["filtered_collective_m_s2"] for row in rows),
        **{key: value for key, value in fit.items() if key not in ("response", "residuals")},
        "quality": quality,
    }
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    if args.require_accepted and not quality["accepted"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
