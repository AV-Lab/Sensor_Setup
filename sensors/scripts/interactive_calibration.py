"""Review a solved LiDAR-camera transform on captured calibration pairs."""

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import sys
import uuid

import cv2
import numpy as np
import yaml

from scripts.calibration_common import CalibrationError
from scripts.calibration_common import default_config_path
from scripts.calibration_common import load_calibration_bundle
from scripts.calibration_common import load_session_pairs
from scripts.calibration_common import project_points
from scripts.calibration_common import read_pcd
from scripts.calibration_common import transform_delta
from scripts.calibration_common import validate_rigid_transform


REFINEMENT_COMMANDS = {
    'tx+': ('translation', 0, 1.0),
    'tx-': ('translation', 0, -1.0),
    'ty+': ('translation', 1, 1.0),
    'ty-': ('translation', 1, -1.0),
    'tz+': ('translation', 2, 1.0),
    'tz-': ('translation', 2, -1.0),
    'roll+': ('rotation', 0, 1.0),
    'roll-': ('rotation', 0, -1.0),
    'pitch+': ('rotation', 1, 1.0),
    'pitch-': ('rotation', 1, -1.0),
    'yaw+': ('rotation', 2, 1.0),
    'yaw-': ('rotation', 2, -1.0),
}


def _rotation_about_axis(axis, angle_radians):
    """Construct a right-handed rotation about one target-frame axis."""
    vector = np.zeros(3, dtype=np.float64)
    vector[axis] = angle_radians
    rotation, _ = cv2.Rodrigues(vector)
    return rotation


def apply_refinement(transform, command, translation_step, rotation_step):
    """Apply an explicit camera-frame translation or rotation increment."""
    if command not in REFINEMENT_COMMANDS:
        raise CalibrationError(f'Unknown refinement command {command!r}.')
    kind, axis, sign = REFINEMENT_COMMANDS[command]
    candidate = validate_rigid_transform(transform).copy()
    if kind == 'translation':
        candidate[axis, 3] += sign * translation_step
    else:
        increment = _rotation_about_axis(axis, sign * rotation_step)
        candidate[:3, :3] = increment @ candidate[:3, :3]
    return validate_rigid_transform(candidate)


def _sample_points(points, maximum):
    """Select a deterministic, uniform subset for responsive display."""
    if points.shape[0] <= maximum:
        return points
    indices = np.linspace(0, points.shape[0] - 1, maximum, dtype=np.int64)
    return points[indices]


def render_overlay(image, points, transform, camera, title_lines=()):
    """Render a distortion-aware depth-coloured cloud on one raw image."""
    pixels, depths, _ = project_points(points, transform, camera)
    if image.shape[1] != camera.width or image.shape[0] != camera.height:
        raise CalibrationError(
            f'Image is {image.shape[1]}x{image.shape[0]}, but intrinsics are '
            f'{camera.width}x{camera.height}.'
        )
    if not pixels.size:
        visible_pixels = np.empty((0, 2), dtype=np.int32)
        visible_depths = np.empty(0)
    else:
        finite = np.isfinite(pixels).all(axis=1)
        visible = finite
        visible &= pixels[:, 0] >= 0.0
        visible &= pixels[:, 0] < camera.width
        visible &= pixels[:, 1] >= 0.0
        visible &= pixels[:, 1] < camera.height
        visible_pixels = np.rint(pixels[visible]).astype(np.int32)
        visible_pixels[:, 0] = np.clip(
            visible_pixels[:, 0], 0, camera.width - 1
        )
        visible_pixels[:, 1] = np.clip(
            visible_pixels[:, 1], 0, camera.height - 1
        )
        visible_depths = depths[visible]

    overlay = image.copy()
    if visible_depths.size:
        lower, upper = np.percentile(visible_depths, [2.0, 98.0])
        scale = max(float(upper - lower), 1e-6)
        normalized = np.clip((visible_depths - lower) / scale, 0.0, 1.0)
        colors = cv2.applyColorMap(
            np.rint(normalized * 255.0).astype(np.uint8),
            cv2.COLORMAP_TURBO,
        ).reshape(-1, 3)
        x = visible_pixels[:, 0]
        y = visible_pixels[:, 1]
        overlay[y, x] = colors
        overlay[np.clip(y + 1, 0, camera.height - 1), x] = colors
        overlay[y, np.clip(x + 1, 0, camera.width - 1)] = colors

    result = cv2.addWeighted(image, 0.55, overlay, 0.75, 0.0)
    lines = list(title_lines) + [f'visible points: {len(visible_depths)}']
    for index, line in enumerate(lines):
        origin = (20, 35 + 30 * index)
        cv2.putText(
            result,
            str(line),
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            result,
            str(line),
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return result


def validate_pair_frames(pair, source_frame, target_frame):
    """Reject a dataset whose message frames contradict the calibration."""
    lidar_frame = pair.get('lidar_frame_id')
    image_frame = pair.get('image_frame_id')
    if lidar_frame != source_frame:
        raise CalibrationError(
            f'Pair {pair["index"]} LiDAR frame is {lidar_frame!r}; '
            f'calibration source is {source_frame!r}.'
        )
    if image_frame != target_frame:
        raise CalibrationError(
            f'Pair {pair["index"]} image frame is {image_frame!r}; '
            f'calibration target is {target_frame!r}.'
        )


def save_candidate(output_directory, bundle, session, original, candidate):
    """Write a non-overwriting refinement candidate with full provenance."""
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc)
    filename = timestamp.strftime(
        'calibration_candidate_%Y%m%dT%H%M%S.%fZ.yaml'
    )
    destination = output_directory / filename
    delta = transform_delta(original, candidate)
    document = {
        'schema_version': 2,
        'candidate': {
            'valid': False,
            'status': 'manual_refinement_requires_acceptance',
            'created_at_utc': timestamp.isoformat(),
            'source_calibration_file': str(bundle.path),
            'capture_session': str(session),
            'source_frame': bundle.extrinsic.source_frame,
            'target_frame': bundle.extrinsic.target_frame,
            'convention': 'p_target = T_target_source * p_source',
            'translation_unit': 'm',
            'original_matrix': original.tolist(),
            'matrix': candidate.tolist(),
            'change_from_original': delta,
        },
    }
    temporary = output_directory / (
        f'.{filename}.tmp-{os.getpid()}-{uuid.uuid4().hex}'
    )
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination, delta


class CalibrationReview:
    """Drive the terminal-controlled offline overlay reviewer."""

    def __init__(self, bundle, session, pairs, mode):
        """Load review policy and initialize one global candidate transform."""
        self.bundle = bundle
        self.session = session
        self.pairs = pairs
        self.mode = mode
        self.original = bundle.extrinsic.matrix.copy()
        self.candidate = self.original.copy()
        review = bundle.raw.get('review', {})
        self.maximum_points = int(review.get('max_display_points', 50000))
        self.translation_step = float(
            review.get('translation_step_m', 0.001)
        )
        self.rotation_step = np.radians(
            float(review.get('rotation_step_deg', 0.05))
        )
        self.warn_translation = float(
            review.get('warn_translation_change_m', 0.02)
        )
        self.warn_rotation = float(
            review.get('warn_rotation_change_deg', 0.5)
        )
        if self.maximum_points <= 0:
            raise CalibrationError(
                'review.max_display_points must be positive.'
            )

    def _load_pair(self, position):
        """Read one manifest-selected image and PCD."""
        pair = self.pairs[position]
        validate_pair_frames(
            pair,
            self.bundle.extrinsic.source_frame,
            self.bundle.extrinsic.target_frame,
        )
        image = cv2.imread(str(pair['image_path']), cv2.IMREAD_COLOR)
        if image is None:
            raise CalibrationError(f'Cannot decode {pair["image_path"]}.')
        points = read_pcd(pair['pointcloud_path'], xyz_only=True)
        return pair, image, _sample_points(points, self.maximum_points)

    def _show(self, position):
        """Display one pair under the current global transform."""
        pair, image, points = self._load_pair(position)
        delta = transform_delta(self.original, self.candidate)
        lines = [
            f'pair {position + 1}/{len(self.pairs)} '
            f'(manifest index {pair["index"]})',
            f'{self.bundle.extrinsic.source_frame} -> '
            f'{self.bundle.extrinsic.target_frame}',
            f'candidate change: {delta["translation_m"] * 1000:.1f} mm, '
            f'{delta["rotation_deg"]:.3f} deg',
        ]
        result = render_overlay(
            image,
            points,
            self.candidate,
            self.bundle.camera,
            lines,
        )
        cv2.imshow('LiDAR-camera calibration review', result)
        cv2.waitKey(1)

    def run(self, start_index=0, output_directory=None):
        """Process terminal commands until the user exits."""
        position = min(max(start_index, 0), len(self.pairs) - 1)
        output_directory = (
            self.session / 'review_results'
            if output_directory is None
            else Path(output_directory)
        )
        cv2.namedWindow(
            'LiDAR-camera calibration review', cv2.WINDOW_NORMAL
        )
        print('Commands: n, p, j <index>, reset, matrix, q')
        if self.mode == 'refine':
            print(
                'Refinement: tx+/-, ty+/-, tz+/-, roll+/-, pitch+/-, '
                'yaw+/-, save'
            )
            print(
                'Translations and rotations are expressed in the target '
                '(camera optical) frame.'
            )
        try:
            while True:
                self._show(position)
                command = input('review> ').strip().lower()
                if command in {'q', 'quit', 'exit'}:
                    break
                if command in {'n', 'next'}:
                    position = min(position + 1, len(self.pairs) - 1)
                elif command in {'p', 'previous'}:
                    position = max(position - 1, 0)
                elif command.startswith('j '):
                    requested = int(command.split(maxsplit=1)[1])
                    position = min(max(requested, 0), len(self.pairs) - 1)
                elif command == 'matrix':
                    print(self.candidate)
                elif command == 'reset':
                    self.candidate = self.original.copy()
                elif command in REFINEMENT_COMMANDS:
                    if self.mode != 'refine':
                        print('Restart with --mode refine to make changes.')
                        continue
                    self.candidate = apply_refinement(
                        self.candidate,
                        command,
                        self.translation_step,
                        self.rotation_step,
                    )
                elif command == 'save':
                    if self.mode != 'refine':
                        print('Review mode cannot save a modified transform.')
                        continue
                    destination, delta = save_candidate(
                        output_directory,
                        self.bundle,
                        self.session,
                        self.original,
                        self.candidate,
                    )
                    print(f'Saved unaccepted candidate: {destination}')
                    if (
                        delta['translation_m'] > self.warn_translation
                        or delta['rotation_deg'] > self.warn_rotation
                    ):
                        print(
                            'WARNING: adjustment exceeds the configured minor '
                            'refinement limit; investigate or recalibrate.'
                        )
                elif command:
                    print(f'Unknown command: {command}')
        finally:
            cv2.destroyAllWindows()


def parse_arguments(arguments=None):
    """Parse command-line options for the offline reviewer."""
    parser = argparse.ArgumentParser(
        description=(
            'Review a solved LiDAR-camera transform on a capture session. '
            'The default mode cannot modify calibration.'
        )
    )
    parser.add_argument(
        '--session', required=True, help='Capture session path'
    )
    parser.add_argument('--config', help='calibrate.yaml path')
    parser.add_argument(
        '--mode', choices=('review', 'refine'), default='review'
    )
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--output-dir')
    return parser.parse_args(arguments)


def main(args=None):
    """Run the calibration review command."""
    options = parse_arguments(args)
    try:
        config = options.config or default_config_path()
        bundle = load_calibration_bundle(config, require_valid=True)
        session, pairs = load_session_pairs(options.session)
        review = CalibrationReview(bundle, session, pairs, options.mode)
        review.run(options.start_index, options.output_dir)
    except (CalibrationError, OSError, ValueError) as error:
        print(f'calibration_review: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
