# This tool is intended to be ran in the OpenMV IDE where it can be used to 
# calibrate the thermal camera vector detection. In the IDE you can see the
# detected blob points, vectors and stats on detections.

# NOTE: when running this via the OpenMV IDE delete the main.py from the device
# and power cycle the device, otherwise the IDE will just freeze or crash

import time
import csi
import image
import math

# Tunable parameters
BLOB_THRESHOLD_LO = 73      # Grayscale value considered "warm enough".
BLOB_THRESHOLD_HI = 255     # Upper bound of the warm range (grayscale max).
MIN_BLOB_PIXELS   = 5       # Minimum hot pixels for a valid blob.
MIN_BLOB_AREA     = 5       # Minimum bounding-box area for a valid blob.
MAX_BLOB_PIXELS   = int(160 * 120) # Maximum pixels per blob (currently no limit as its the same as full frame)

MIN_MOTION_PX        = 5    # Min displacement (px) between two frames to count as moving.
MAX_MATCH_DISTANCE   = 20   # Max px a blob may jump and still match the same track.
TRAIL_LENGTH         = 5    # Keep motion vectors for the last N frames.
RECORD_MIN_VECTORS   = 3    # Need at least this many linear moving vectors to record.
CURVATURE_LIMIT      = 0.15 # Max (deviation / path length) ratio to count as "mostly linear".

# Camera setup
alpha_pal = image.Image(256, 1, image.GRAYSCALE)
for i in range(256):
    alpha_pal[i] = int(math.pow((i / 255), 2) * 255)

# Color camera.
csi0 = csi.CSI()
csi0.reset(hard=True)        # force hardware reset.
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QQVGA)

# FLIR Lepton thermal camera
csi1 = csi.CSI(cid=csi.LEPTON)
csi1.reset(hard=False)        # no hardware reset - just configure lepton.
csi1.pixformat(csi.GRAYSCALE)
csi1.framesize(csi.QQVGA)

# Optional temperature range controls for the LEPTON.
csi1.ioctl(csi.IOCTL_LEPTON_SET_MODE, True, False)
# Tune thermal camera temperature range here
csi1.ioctl(csi.IOCTL_LEPTON_SET_RANGE, 20.0, 40.0)

clock = time.clock()

LW, LH = csi1.width(), csi1.height()
CW, CH = csi0.width(), csi0.height()
SX = CW / LW
SY = CH / LH

# img0 is provided fresh by csi0.snapshot() each loop (it wraps the camera
# frame buffer, so drawing on it is what gets sent to the IDE preview).
img1 = image.Image(LW, LH, csi1.pixformat())   # lepton grayscale working image.

# Prime the color camera with one initial frame before the loop.
csi0.snapshot()

# Tracking state. Each track is a dict:
#   trail : list of (x, y) centroids, most recent first, capped at TRAIL_LENGTH+1.
tracks = []


def dist(ax, ay, bx, by):
    dx = ax - bx
    dy = ay - by
    return math.sqrt(dx * dx + dy * dy)


def find_blobs(img):
    """Return the list of warm blobs in the thermal frame."""
    return img.find_blobs(
        [(BLOB_THRESHOLD_LO, BLOB_THRESHOLD_HI)],
        pixels_threshold=MIN_BLOB_PIXELS,
        area_threshold=MIN_BLOB_AREA,
        merge=True
    )


def update_tracks(blobs):
    """Match this frame's blobs to existing tracks, advance trails,
    and start tracks for unmatched blobs.
    """
    global tracks

    matched = set()
    new_tracks = []

    for tr in tracks:
        trail = tr.get("trail")
        if not trail:
            continue

        lastx, lasty = trail[0]
        best = None
        best_d = MAX_MATCH_DISTANCE

        for i, blob in enumerate(blobs):
            if i in matched:
                continue

            d = dist(lastx, lasty, blob.cx, blob.cy)
            if d <= best_d:
                best_d = d
                best = i

        if best is not None:
            matched.add(best)
            trail.insert(0, (blobs[best].cx, blobs[best].cy))
            del trail[TRAIL_LENGTH + 1:]
            new_tracks.append(tr)

    # Start a track for every blob not assigned to an existing track.
    for i, blob in enumerate(blobs):
        if i not in matched:
            new_tracks.append({
                "trail": [(blob.cx, blob.cy)]
            })

    tracks = new_tracks


def is_moving(tr):
    """A track is moving if it has two points and they moved by >= MIN_MOTION_PX."""
    if len(tr['trail']) < 2:
        return False
    (x0, y0), (x1, y1) = tr['trail'][0], tr['trail'][1]
    return dist(x0, y0, x1, y1) >= MIN_MOTION_PX


def is_linear(tr):
    """True if the track's recent path is mostly straight (small curve only)."""
    trail = tr['trail']
    if len(trail) < 3:
        return False
    p0 = trail[-1]   # oldest point
    pn = trail[0]    # newest point
    dx = pn[0] - p0[0]
    dy = pn[1] - p0[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length == 0 or length < MIN_MOTION_PX:
        return False
    max_dev = 0.0
    for p in trail[1:-1]:
        # Perpendicular distance from point p to the line p0 -> pn.
        num = abs(dy * (p[0] - p0[0]) - dx * (p[1] - p0[1]))
        dev = num / length
        if dev > max_dev:
            max_dev = dev
    return (max_dev / length) <= CURVATURE_LIMIT


def to_display(x, y):
    """The thermal image is drawn at native size in the top-left, so detection
    coordinates map straight onto the display frame."""
    return int(x * SX), int(y * SY)

# Main loop
frame_count = 0
while True:
    time.sleep(1 / 5)

    clock.tick()
    img0 = csi0.snapshot()

    frame_count += 1

    csi1.snapshot(blocking=False, image=img1)
    img0.draw_image(img1, 0, 0, color_palette=image.PALETTE_IRONBOW,
                    alpha_palette=alpha_pal,
                    hint=image.BILINEAR)
    blobs = find_blobs(img1)
    update_tracks(blobs)

    linear_count = 0
    for tr in tracks:
        if not is_moving(tr):
            continue
        if is_linear(tr):
            linear_count += 1
        trail = tr['trail']
        for i in range(len(trail) - 1):
            ax, ay = to_display(trail[i][0], trail[i][1])
            bx, by = to_display(trail[i + 1][0], trail[i + 1][1])
            img0.draw_line((ax, ay, bx, by), color=(255, 0, 0), thickness=2)

    for b in blobs:
        cx, cy = to_display(b.cx, b.cy)
        img0.draw_cross((cx, cy), color=(255, 0, 0))

    # report stats
    # fps here is the maximum fps this loop can achieve, not he actual running fps
    print("fps={:.1f} blobs={} moving={} linear={}".format(
        clock.fps(), len(blobs), sum(1 for t in tracks if is_moving(t)),
        linear_count))
