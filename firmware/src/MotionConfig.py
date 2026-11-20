class MotionConfig:
    def __init__(self):
        self._chk_mot_ms = 200 # 5 fps
        self._init_file_num = -1  # Initial file counter value
        self._motion_timeout_ms = 5000  # 5 seconds

        self._blob_threshold_lo = 73   # Grayscale considered "warm enough".
        self._blob_threshold_hi = 255  # Upper bound of the warm range (grayscale max).
        self._min_temp_diff = 2        # Hottest point must be this many °C above the scene average.
        self._min_blob_pixels = 5      # Minimum hot pixels for a valid target blob.
        self._min_blob_area = 5        # Minimum bounding-box area for a valid target blob.
        self._max_blob_pixels = int(160 * 120)

        self._min_motion_px = 5         # Min displacement (px) between two frames to count as moving.
        self._max_match_distance = 20   # Max px a blob may jump and still match the same track.
        self._trail_length = 5          # Keep motion vectors for the last N frames.
        self._record_min_vectors = 3    # Need at least this many linear moving vectors to record.
        self._curvature_limit = 0.3     # Max (deviation / path length) ratio to count as "mostly linear".

        self._min_temp_in_celsius = 20.0  # Minimum temperature represented by grayscale value 0.
        self._max_temp_in_celsius = 40.0  # Maximum temperature represented by grayscale value 255.

    def chk_mot_ms(self):
        return self._chk_mot_ms

    def init_file_num(self):
        return self._init_file_num

    def motion_timeout_ms(self):
        return self._motion_timeout_ms

    def blob_threshold_lo(self):
        return self._blob_threshold_lo

    def blob_threshold_hi(self):
        return self._blob_threshold_hi

    def min_temp_diff(self):
        return self._min_temp_diff

    def min_blob_pixels(self):
        return self._min_blob_pixels

    def min_blob_area(self):
        return self._min_blob_area

    def max_blob_pixels(self):
        return self._max_blob_pixels

    def min_motion_px(self):
        return self._min_motion_px

    def max_match_distance(self):
        return self._max_match_distance

    def trail_length(self):
        return self._trail_length

    def record_min_vectors(self):
        return self._record_min_vectors

    def curvature_limit(self):
        return self._curvature_limit

    def min_temp_in_celsius(self):
        return self._min_temp_in_celsius

    def max_temp_in_celsius(self):
        return self._max_temp_in_celsius
