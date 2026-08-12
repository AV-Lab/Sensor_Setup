"""Conservatively screen LiDAR clouds for a checkerboard-like plane."""

import numpy as np

from scripts.calibration_common import CalibrationError


def _number(mapping, key, default, minimum=None, maximum=None):
    """Read one finite detector setting with optional inclusive bounds."""
    value = mapping.get(key, default)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
    ):
        raise CalibrationError(
            f'lidar_target_detection.{key} must be a finite number.'
        )
    value = float(value)
    if minimum is not None and value < minimum:
        raise CalibrationError(
            f'lidar_target_detection.{key} must be at least {minimum}.'
        )
    if maximum is not None and value > maximum:
        raise CalibrationError(
            f'lidar_target_detection.{key} must be at most {maximum}.'
        )
    return value


def _integer(mapping, key, default, minimum=1):
    """Read one non-boolean integer detector setting."""
    value = mapping.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise CalibrationError(
            f'lidar_target_detection.{key} must be an integer.'
        )
    if value < minimum:
        raise CalibrationError(
            f'lidar_target_detection.{key} must be at least {minimum}.'
        )
    return value


def load_lidar_detection_policy(document):
    """Validate the optional conservative LiDAR target detector policy."""
    if document is None:
        document = {}
    if not isinstance(document, dict):
        raise CalibrationError('lidar_target_detection must be a mapping.')
    for key in ('enabled', 'required'):
        if not isinstance(document.get(key, False), bool):
            raise CalibrationError(
                f'lidar_target_detection.{key} must be true or false.'
            )
    if document.get('required', False) and not document.get('enabled', False):
        raise CalibrationError(
            'lidar_target_detection.required needs enabled=true.'
        )

    minimum_range = _number(document, 'min_range_m', 0.5, minimum=0.0)
    maximum_range = _number(document, 'max_range_m', 8.0, minimum=0.01)
    if maximum_range <= minimum_range:
        raise CalibrationError(
            'lidar_target_detection.max_range_m must exceed min_range_m.'
        )
    return {
        'enabled': document.get('enabled', False),
        'required': document.get('required', False),
        'min_range_m': minimum_range,
        'max_range_m': maximum_range,
        'plane_distance_threshold_m': _number(
            document,
            'plane_distance_threshold_m',
            0.02,
            minimum=0.001,
        ),
        'minimum_points': _integer(
            document, 'minimum_points', 30, minimum=6
        ),
        'seed_percentile': _number(
            document,
            'seed_percentile',
            80.0,
            minimum=0.0,
            maximum=100.0,
        ),
        'max_seeds': _integer(document, 'max_seeds', 64),
        'ransac_hypotheses': _integer(
            document, 'ransac_hypotheses', 24
        ),
        'neighborhood_radius_scale': _number(
            document,
            'neighborhood_radius_scale',
            0.85,
            minimum=0.25,
            maximum=2.0,
        ),
        'extent_tolerance_ratio': _number(
            document,
            'extent_tolerance_ratio',
            0.35,
            minimum=0.05,
            maximum=1.0,
        ),
        'min_checker_contrast': _number(
            document,
            'min_checker_contrast',
            0.30,
            minimum=0.0,
            maximum=1.0,
        ),
        'min_checker_accuracy': _number(
            document,
            'min_checker_accuracy',
            0.62,
            minimum=0.5,
            maximum=1.0,
        ),
        'min_occupied_cell_fraction': _number(
            document,
            'min_occupied_cell_fraction',
            0.25,
            minimum=0.0,
            maximum=1.0,
        ),
    }


def _fit_local_plane(points, threshold, hypotheses, random_generator):
    """Find a locally dominant plane, then refine it using PCA."""
    if len(points) < 3:
        return None
    best_mask = None
    best_count = 0
    for _ in range(hypotheses):
        indices = random_generator.choice(len(points), 3, replace=False)
        first, second, third = points[indices]
        normal = np.cross(second - first, third - first)
        magnitude = np.linalg.norm(normal)
        if magnitude < 1e-9:
            continue
        normal /= magnitude
        distances = np.abs((points - first) @ normal)
        mask = distances <= threshold
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < 3:
        return None

    inliers = points[best_mask]
    center = np.mean(inliers, axis=0)
    _, _, axes = np.linalg.svd(inliers - center, full_matrices=False)
    normal = axes[2]
    distances = np.abs((points - center) @ normal)
    refined_mask = distances <= threshold
    refined = points[refined_mask]
    if len(refined) < 3:
        return None
    center = np.mean(refined, axis=0)
    _, _, axes = np.linalg.svd(refined - center, full_matrices=False)
    normal = axes[2]
    distances = np.abs((refined - center) @ normal)
    return refined_mask, center, axes, float(np.sqrt(np.mean(distances ** 2)))


def _checker_metrics(projected, values, square_counts):
    """Measure alternating scalar structure on a projected plane patch."""
    lower = np.percentile(projected, 1.0, axis=0)
    upper = np.percentile(projected, 99.0, axis=0)
    extents = upper - lower
    if np.any(extents <= 1e-6):
        return None

    if extents[0] < extents[1]:
        projected = projected[:, ::-1]
        lower = lower[::-1]
        upper = upper[::-1]
        extents = extents[::-1]

    inside = np.logical_and(projected >= lower, projected <= upper).all(axis=1)
    projected = projected[inside]
    values = values[inside]
    normalized = (projected - lower) / extents
    cell_indices = np.floor(
        normalized * np.asarray(square_counts, dtype=np.float64)
    ).astype(int)
    cell_indices = np.minimum(
        cell_indices,
        np.asarray(square_counts, dtype=int) - 1,
    )
    parity = (cell_indices[:, 0] + cell_indices[:, 1]) % 2
    first = values[parity == 0]
    second = values[parity == 1]
    if len(first) < 3 or len(second) < 3:
        return None

    low_value, high_value = np.percentile(values, [5.0, 95.0])
    dynamic_range = float(high_value - low_value)
    if dynamic_range <= 1e-6:
        return None
    first_mean = float(np.mean(first))
    second_mean = float(np.mean(second))
    contrast = min(abs(first_mean - second_mean) / dynamic_range, 1.0)
    threshold = 0.5 * (first_mean + second_mean)
    bright_first = values >= threshold
    expected_first = parity == 0
    accuracy = max(
        float(np.mean(bright_first == expected_first)),
        float(np.mean(bright_first != expected_first)),
    )
    occupied = np.unique(cell_indices, axis=0).shape[0]
    occupied_fraction = occupied / float(np.prod(square_counts))
    return {
        'extents_m': extents,
        'checker_contrast': contrast,
        'checker_classification_accuracy': accuracy,
        'occupied_cell_fraction': occupied_fraction,
        'evaluated_point_count': int(len(values)),
    }


def _empty_result(status, value_field, reason):
    """Create a stable detector result for a non-candidate outcome."""
    return {
        'status': status,
        'value_field': value_field,
        'reason': reason,
        'requires_manual_confirmation': True,
        'candidate': None,
    }


def detect_lidar_checkerboard(xyz, values, target, policy, value_field):
    """
    Find the best planar checkerboard-like candidate in one cloud.

    This is deliberately a screening detector. A result with status
    ``candidate`` still requires visual or solver-side confirmation.
    """
    if not policy['enabled']:
        return _empty_result('disabled', value_field, 'detector is disabled')
    if value_field not in ('reflectivity', 'intensity', 'signal'):
        return _empty_result(
            'unavailable',
            value_field,
            'cloud has no reflectivity-like scalar field',
        )

    xyz = np.asarray(xyz, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) != len(values):
        raise ValueError('xyz must have shape (N, 3) and match values.')
    ranges = np.linalg.norm(xyz, axis=1)
    valid = np.isfinite(xyz).all(axis=1) & np.isfinite(values)
    valid &= ranges >= policy['min_range_m']
    valid &= ranges <= policy['max_range_m']
    xyz = xyz[valid]
    values = values[valid]
    if len(xyz) < policy['minimum_points']:
        return _empty_result(
            'not_detected', value_field, 'too few range-filtered points'
        )

    inner_columns, inner_rows = target['inner_corners']
    square_size = float(target['square_size_m'])
    square_counts = tuple(sorted(
        (inner_columns + 1, inner_rows + 1), reverse=True
    ))
    expected_extents = np.asarray(square_counts, dtype=np.float64) * square_size
    expected_extents = np.sort(expected_extents)[::-1]
    board_diagonal = float(np.linalg.norm(expected_extents))
    neighborhood_radius = (
        board_diagonal * policy['neighborhood_radius_scale']
    )

    seed_threshold = np.percentile(values, policy['seed_percentile'])
    seed_indices = np.flatnonzero(values >= seed_threshold)
    if len(seed_indices) > policy['max_seeds']:
        selection = np.linspace(
            0, len(seed_indices) - 1, policy['max_seeds'], dtype=int
        )
        seed_indices = seed_indices[selection]

    random_generator = np.random.default_rng(0)
    best = None
    for seed_index in seed_indices:
        local_mask = (
            np.linalg.norm(xyz - xyz[seed_index], axis=1)
            <= neighborhood_radius
        )
        local_points = xyz[local_mask]
        local_values = values[local_mask]
        if len(local_points) < policy['minimum_points']:
            continue
        plane = _fit_local_plane(
            local_points,
            policy['plane_distance_threshold_m'],
            policy['ransac_hypotheses'],
            random_generator,
        )
        if plane is None:
            continue
        plane_mask, center, axes, rmse = plane
        plane_points = local_points[plane_mask]
        plane_values = local_values[plane_mask]
        if len(plane_points) < policy['minimum_points']:
            continue
        projected = (plane_points - center) @ axes[:2].T
        metrics = _checker_metrics(projected, plane_values, square_counts)
        if metrics is None:
            continue

        extents = np.sort(metrics['extents_m'])[::-1]
        extent_errors = np.abs(extents / expected_extents - 1.0)
        score = (
            metrics['checker_contrast']
            + metrics['checker_classification_accuracy']
            + metrics['occupied_cell_fraction']
            - float(np.max(extent_errors))
            - rmse / policy['plane_distance_threshold_m']
        )
        candidate = {
            'score': float(score),
            'center_sensor_m': center.astype(float).tolist(),
            'normal_sensor': axes[2].astype(float).tolist(),
            'plane_axes_sensor': axes[:2].astype(float).tolist(),
            'plane_rmse_m': rmse,
            'plane_point_count': int(len(plane_points)),
            'estimated_extents_m': extents.astype(float).tolist(),
            'expected_extents_m': expected_extents.astype(float).tolist(),
            'maximum_extent_error_ratio': float(np.max(extent_errors)),
            **{
                key: value
                for key, value in metrics.items()
                if key != 'extents_m'
            },
        }
        candidate['passes_thresholds'] = bool(
            np.max(extent_errors) <= policy['extent_tolerance_ratio']
            and rmse <= policy['plane_distance_threshold_m']
            and metrics['checker_contrast']
            >= policy['min_checker_contrast']
            and metrics['checker_classification_accuracy']
            >= policy['min_checker_accuracy']
            and metrics['occupied_cell_fraction']
            >= policy['min_occupied_cell_fraction']
        )
        if best is None or candidate['score'] > best['score']:
            best = candidate

    if best is None:
        return _empty_result(
            'not_detected', value_field, 'no planar checker-pattern patch found'
        )
    if not best['passes_thresholds']:
        result = _empty_result(
            'not_detected',
            value_field,
            'best patch did not pass all configured thresholds',
        )
        result['candidate'] = best
        return result
    return {
        'status': 'candidate',
        'value_field': value_field,
        'reason': (
            'planar patch has expected scale and alternating scalar pattern'
        ),
        'requires_manual_confirmation': True,
        'candidate': best,
    }
