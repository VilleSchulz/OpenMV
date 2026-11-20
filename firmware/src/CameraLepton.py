from src.CameraBase import Camera
import csi
import time
import math
import image
import ustruct
import asyncio

class CameraLepton(Camera):
    def __init__(self, log_manager):
        super().__init__()
        self.name = "Lepton-3.5"

        self._log_manager = log_manager

        self._last_motion_check_time = time.ticks_ms()

        # Initialize the FLIR Lepton thermal camera second.
        # Use a soft reset so the already initialized PAG7936 remains active.
        self.csi1 = csi.CSI(cid=csi.LEPTON, stream=False)
        self.csi1.reset(hard=False)  # Soft reset and initialize the sensor
        self.csi1.pixformat(csi.GRAYSCALE)  # Set pixel format to GRAYSCALE
        self.csi1.framesize(csi.QQVGA)  # Native Lepton resolution: 160x120

        # Enable radiometric measurement mode and map the grayscale output
        # to the configured temperature range.
        self.csi1.ioctl(csi.IOCTL_LEPTON_SET_MODE, True, False)
        self.csi1.ioctl(
            csi.IOCTL_LEPTON_SET_RANGE,
            self._mot_conf.min_temp_in_celsius(),
            self._mot_conf.max_temp_in_celsius()
        )

        # Allow the Lepton to stabilize before capturing the first frame.
        self._current_frame = (
            self.csi1.snapshot(time=self.cam_config.lepton_stabilization_ms())
        )

        # tracks used for vector detection
        self._tracks = []

        self._ffc_active = False
        self._ffc_recovery_until = None
        self._last_ffc_check_time = 0
        self._ffc_check_interval_ms = 1000
        self._ffc_recalibration_time_ms = 5000

        self._lepton_ffc_status_attribute = 0x0244
        self._lepton_ffc_status_word_count = 2

        self.filename_lepton = None
        self.video_lepton = None
        self.saved_frames = 0

        self._tools.print_memory_status("After Lepton init")

    def _write_to_lepton(self, frame):
        try:
            self.video_lepton.write(frame)
        except Exception as error:
            self._log_manager.error("SD card write failed: {}".format(error))
            raise

    def record_frame(self):
        self._current_frame = self.csi1.snapshot()
        self._write_to_lepton(self._current_frame)
        self.saved_frames += 1

    def _detect_motion(self):
        img = self._current_frame
        if not img:
            return False

        blobs = self._find_blobs(img)
        self._update_tracks(blobs)

        linear_count = 0
        for tr in self._tracks:
            if not self._is_moving(tr):
                continue
            if self._is_linear(tr):
                linear_count += 1

        # enable record when there is linear motion
        return linear_count > 0

    def _find_blobs(self, img):
        return img.find_blobs(
            [(self._mot_conf.blob_threshold_lo(), self._mot_conf.blob_threshold_hi())],
            pixels_threshold=self._mot_conf.min_blob_pixels(),
            area_threshold=self._mot_conf.min_blob_area(),
            merge=True
        )

    def _dist(self, ax, ay, bx, by):
        dx = ax - bx
        dy = ay - by
        return math.sqrt(dx * dx + dy * dy)

    def _update_tracks(self, blobs):
        matched = set()
        new_tracks = []

        for tr in self._tracks:
            trail = tr.get("trail")
            if not trail:
                continue

            lastx, lasty = trail[0]
            best = None
            best_d = self._mot_conf.max_match_distance()

            for i, blob in enumerate(blobs):
                if i in matched:
                    continue

                d = self._dist(lastx, lasty, blob.cx, blob.cy)
                if d <= best_d:
                    best_d = d
                    best = i

            if best is not None:
                matched.add(best)
                trail.insert(0, (blobs[best].cx, blobs[best].cy))
                del trail[self._mot_conf.trail_length() + 1:]
                new_tracks.append(tr)

        # Start a track for every blob not assigned to an existing track.
        for i, blob in enumerate(blobs):
            if i not in matched:
                new_tracks.append({
                    "trail": [(blob.cx, blob.cy)]
                })

        self._tracks = new_tracks

    def _is_moving(self, tr):
        if len(tr['trail']) < 2:
            return False
        (x0, y0), (x1, y1) = tr['trail'][0], tr['trail'][1]
        return self._dist(x0, y0, x1, y1) >= self._mot_conf.min_motion_px()


    def _is_linear(self, tr):
        trail = tr['trail']
        if len(trail) < 3:
            return False
        p0 = trail[-1]   # oldest point
        pn = trail[0]    # newest point
        dx = pn[0] - p0[0]
        dy = pn[1] - p0[1]
        length = math.sqrt(dx * dx + dy * dy)
        if length == 0 or length < self._mot_conf.min_motion_px():
            return False
        max_dev = 0.0
        for p in trail[1:-1]:
            # Perpendicular distance from point p to the line p0 -> pn.
            num = abs(dy * (p[0] - p0[0]) - dx * (p[1] - p0[1]))
            dev = num / length
            if dev > max_dev:
                max_dev = dev
        return (max_dev / length) <= self._mot_conf.curvature_limit()

    def detect_motion_lepton(self):
        return self._detect_motion()

    async def detect_motion_async_lepton(self):
        return self._detect_motion()

    def write_prebuffer_with_catchup_lepton(self):
        last_live_frame_time_lepton = time.ticks_ms()

        self.saved_frames = 0

        # Retrieve the buffered frames in chronological order.
        prebuf_frames_lepton, self._buffer_index = (
            self._get_ordered_buf_frames(self._buffer, self._buffer_index)
        )

        # Stores frames captured while the pre-buffer is being written.
        # These frames are appended afterwards to reduce the recording gap.
        catchup_frames_lepton = []

        # Write the buffered frames to the MJPEG file.
        for frame in prebuf_frames_lepton:
            self._write_to_lepton(frame)
            self.saved_frames += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, last_live_frame_time_lepton) >= self._frame_interval_ms:
                self._current_frame = self.csi1.snapshot()
                catchup_frames_lepton.append(self._current_frame.copy())
                last_live_frame_time_lepton = now

        # Append the frames captured during the pre-buffer write so the
        # transition from buffered video to live recording is as seamless as possible.
        for frame in catchup_frames_lepton:
            self._write_to_lepton(frame)
            self.saved_frames += 1

    async def update_frame_buffer_lepton(self):
        while True:
            try:
                self._current_frame = await self._snapshot_async(self.csi1)
                self._buffer_index = (
                    self._save_frame(
                        self._current_frame.copy(),
                        self._buffer,
                        self._buffer_index,
                    )
                )
            except Exception as error:
                self._log_manager.error("Lepton frame-buffer error: {}".format(error))
                print("Lepton frame-buffer error:", error)
            await asyncio.sleep_ms(self._buf_config.frame_interval_ms())

    # FFC (Flat-Field Correction) is an internal calibration process performed
    # periodically by the FLIR Lepton thermal camera to compensate for sensor drift.
    # FFC temporarily changes the thermal image and can therefore create large
    # differences between consecutive frames, which frame-difference motion
    # detection could incorrectly interpret as movement.
    #
    # The functions below monitor the Lepton FFC state, suspend thermal motion
    # detection while calibration is active, wait for the image to stabilize,
    # and then replace the old frame-difference background with a new reference.
    def _get_ffc_status(self):
        # Read the Lepton FFC status attribute.
        # 0x0244 identifies the FFC status attribute and 2 requests
        # two 16-bit words (32 bits) from the Lepton.
        data = self.csi1.ioctl(
            csi.IOCTL_LEPTON_GET_ATTRIBUTE,
            self._lepton_ffc_status_attribute,
            self._lepton_ffc_status_word_count
        )
        # Convert the four returned bytes into a 32-bit little-endian integer.
        return ustruct.unpack("<I", data)[0]

    def handle_ffc(self):
        # Return True whenever thermal motion detection should be skipped.
        # This includes the FFC itself and the post-FFC stabilization period.
        now = time.ticks_ms()
        # This method may be called several times per second by motion detection,
        # but query the Lepton FFC status only once per configured interval.
        if time.ticks_diff(now, self._last_ffc_check_time) >= self._ffc_check_interval_ms:
            self._last_ffc_check_time = now
            ffc_status = self._get_ffc_status()
            # A non-zero status means that the Lepton is currently performing FFC.
            # Motion detection must be skipped because FFC changes the thermal image
            # and could otherwise be interpreted as movement.
            if ffc_status != 0:
                if not self._ffc_active:
                    self._ffc_active = True
                return True

            # If FFC was active during the previous poll but is no longer active,
            # the calibration has just finished.
            if self._ffc_active:
                self._ffc_active = False
                # Give the thermal image five seconds to stabilize before
                # allowing blob detection to resume.
                self._ffc_recovery_until = time.ticks_add(now, self._ffc_recalibration_time_ms)
                return True
        # FFC status is polled less frequently than this method is called.
        # Keep motion detection disabled between polls while the last known
        # Lepton state says that FFC is still active.
        if self._ffc_active:
            return True
        # Keep motion detection disabled during the post-FFC recovery period.
        if self._ffc_recovery_until is not None:
            if time.ticks_diff(self._ffc_recovery_until, now) > 0:
                return True
            # Recovery has finished; allow blob detection to resume.
            self._ffc_recovery_until = None
            # Skip the current motion check so the first detection after
            # recovery starts from a stable thermal image.
            return True
        # False means that no FFC or recovery is active and thermal
        # blob detection can safely proceed.
        return False

    def prepare_video(self, file_manager):
        # Create a new MJPEG file and prepare the camera for recording.
        self.filename_lepton, self.video_lepton = self._create_motion_video(
            file_manager,
            self._storage_config.video_prefix_lepton(),
            self.cam_config.width_lepton(),
            self.cam_config.height_lepton()
        )

    def finalize_video(self, file_manager):
        self.video_lepton.close()
        duration_ms = self.saved_frames * self.frame_interval_ms()
        file_manager.patch_mjpeg_timing(
            self.filename_lepton,
            self.saved_frames,
            duration_ms
        )
        file_manager.patch_mjpeg_index(self.filename_lepton)

    def frame_interval_ms(self):
        return self._frame_interval_ms