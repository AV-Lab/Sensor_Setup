"""Runtime diagnostics for the camera publisher."""

import time


class CameraDiagnostics:
    """Track camera capture failures, frame rate, and frame periods."""

    def __init__(
        self,
        logger,
        requested_fps,
        timestamp_source,
        interval_sec,
        max_consecutive_failures,
    ):
        """Initialize diagnostic counters and reporting thresholds."""
        self._logger = logger
        self._requested_fps = float(requested_fps)
        self._timestamp_source = timestamp_source
        self._interval_sec = float(interval_sec)
        self._max_consecutive_failures = max_consecutive_failures

        self._published_frames = 0
        self._capture_failures = 0
        self._consecutive_failures = 0
        self._diagnostic_start_time = time.monotonic()
        self._diagnostic_start_frames = 0
        self._previous_receipt_ns = None
        self._reset_period_statistics()

    def record_capture_failure(self, reason):
        """Record failure and return whether the limit was reached."""
        self._capture_failures += 1
        self._consecutive_failures += 1
        self._logger.warning(
            f'Camera capture failure {self._consecutive_failures}/'
            f'{self._max_consecutive_failures}: {reason}'
        )
        return self._consecutive_failures >= self._max_consecutive_failures

    def record_capture_success(self):
        """Reset the consecutive-failure counter after a valid frame."""
        self._consecutive_failures = 0

    def record_published_frame(self, receipt_ns):
        """Update frame timing statistics and report them periodically."""
        self._published_frames += 1
        if self._previous_receipt_ns is not None:
            period_ms = (receipt_ns - self._previous_receipt_ns) / 1_000_000.0
            self._period_count += 1
            self._period_sum_ms += period_ms
            self._period_min_ms = min(self._period_min_ms, period_ms)
            self._period_max_ms = max(self._period_max_ms, period_ms)
        self._previous_receipt_ns = receipt_ns

        now = time.monotonic()
        elapsed = now - self._diagnostic_start_time
        if elapsed < self._interval_sec:
            return

        interval_frames = (
            self._published_frames - self._diagnostic_start_frames
        )
        measured_fps = interval_frames / elapsed
        message = self._make_report(measured_fps)

        if abs(measured_fps - self._requested_fps) > self._requested_fps * 0.1:
            self._logger.warning(message)
        else:
            self._logger.info(message)

        self._diagnostic_start_time = now
        self._diagnostic_start_frames = self._published_frames
        self._reset_period_statistics()

    def _make_report(self, measured_fps):
        """Format the current diagnostic report."""
        if self._period_count:
            average_period_ms = self._period_sum_ms / self._period_count
            period_text = (
                f'period_ms avg={average_period_ms:.3f}, '
                f'min={self._period_min_ms:.3f}, '
                f'max={self._period_max_ms:.3f}'
            )
        else:
            period_text = 'period_ms unavailable'

        return (
            f'Camera diagnostics: fps={measured_fps:.3f}, {period_text}, '
            f'published={self._published_frames}, '
            f'capture_failures={self._capture_failures}, '
            f'timestamp_source={self._timestamp_source}'
        )

    def _reset_period_statistics(self):
        """Reset the statistics accumulated during one reporting interval."""
        self._period_count = 0
        self._period_sum_ms = 0.0
        self._period_min_ms = float('inf')
        self._period_max_ms = 0.0
