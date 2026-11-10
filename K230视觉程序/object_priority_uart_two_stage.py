from libs.YOLO import YOLO26
from libs.Utils import ScopedTiming
from ybUtils.YbUart import YbUart
from media.sensor import *
from media.media import MediaManager
import gc
import sys


# Copy the seven-class YOLO26 detection KModel to this path on the K230.
DETECT_KMODEL_PATH = "/data/best.kmodel"
LABELS = [
    "zhengfangti",
    "yuanzhu",
    "changfangti",
    "lengzhu",
    "qiu",
    "sanlengzhui",
    "yuanzhui",
]
GRAB_PRIORITY = [
    "zhengfangti",
    "yuanzhu",
    "lengzhu",
    "qiu",
    "changfangti",

]

# LAB thresholds: (L_min, L_max, a_min, a_max, b_min, b_max).
COLOR_THRESHOLDS = [
    ("red",        (35, 60,   40, 75,    18, 39)),
    ("black",      ( 0, 28,  -18, 18,   -18, 18)),
    ("yellow",     (55, 100, -25, 15,    40, 100)),
    ("dark blue",  ( 8, 45,   -5, 70,  -100, -15)),
    ("light blue", (45, 90,  -40, 25,   -70, -10)),
    ("orange",     (45, 80,   35, 65,    40, 75)),
    ("green",      (20, 85,  -80, -15,    5, 70)),
]
COLOR_SAMPLE_GRID = 6

# Must match PT training/export and ONNX-to-KModel conversion dimensions.
MODEL_INPUT_SIZE = [640, 640]
RGB888P_SIZE = [640, 480]

CONFIDENCE_THRESHOLD = 0.7
MAX_BOXES_NUM = 20
TIMING_DEBUG = 0

INFERENCE_INTERVAL_FRAMES = 1
GC_INTERVAL_FRAMES = 120
CAMERA_WARMUP_FRAMES = 10
STABLE_FRAMES = 5
MAX_CENTER_SPREAD_PX = 6
TRACK_MATCH_DISTANCE_PX = 30
UART_BAUDRATE = 115200
START_COMMAND = b"s"
# Nonlinear least-squares homography from ten upright-camera pairs.
# Input is camera pixel (u, v), with the origin at the image top-left.
# Output is gripper-plane (X, Y) in centimeters.
PIXEL_TO_GRIPPER = (
    0.0152267823057697, 0.207001091804612, -30.1588935511892,
    -0.156495756247987, 0.0253544668755146, 61.9215783218628,
    0.00174501303854493, 0.00500091624000146, 1.0,
)
GRIPPER_X_MIN_CM = 0.0
GRIPPER_X_MAX_CM = 17
GRIPPER_Y_MIN_CM = 0.0
GRIPPER_Y_MAX_CM = 13
GRIPPER_X_OFFSET_MM = -10
GRIPPER_y_OFFSET_MM = 3.8
GRIPPER_X_LOW_COMPENSATION_CM = 5.88
GRIPPER_X_HIGH_COMPENSATION_CM = 13.10
GRIPPER_X_OUTER_OFFSET_MM = 6
GRIPPER_X_OUTER_OFFSET_MM0 = 6


class HeadlessCamera:
    """Capture the RGB AI channel without initializing Display or OSD."""

    def __init__(self, frame_size):
        self.frame_size = frame_size
        self.sensor = None
        self.media_initialized = False
        self.current_frame = None

    def create(self):
        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(
            w=self.frame_size[0], h=self.frame_size[1],
            chn=CAM_CHN_ID_2
        )
        self.sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
        MediaManager.init()
        self.media_initialized = True
        self.sensor.run()

    def get_frame(self):
        self.current_frame = self.sensor.snapshot(chn=CAM_CHN_ID_2)
        return self.current_frame.to_numpy_ref()

    def destroy(self):
        self.current_frame = None
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except BaseException as error:
                print("Sensor stop failed:", error)
            self.sensor = None
        if self.media_initialized:
            try:
                MediaManager.deinit()
            except BaseException as error:
                print("MediaManager deinit failed:", error)
            self.media_initialized = False


def rgb_to_lab(red, green, blue):
    """Convert one sRGB pixel to LAB with minimal temporary allocation."""
    red = float(red) / 255.0
    green = float(green) / 255.0
    blue = float(blue) / 255.0
    red = (red / 12.92 if red <= 0.04045
           else ((red + 0.055) / 1.055) ** 2.4)
    green = (green / 12.92 if green <= 0.04045
             else ((green + 0.055) / 1.055) ** 2.4)
    blue = (blue / 12.92 if blue <= 0.04045
            else ((blue + 0.055) / 1.055) ** 2.4)

    x_value = ((0.4124564*red + 0.3575761*green +
                0.1804375*blue) / 0.95047)
    y_value = 0.2126729*red + 0.7151522*green + 0.0721750*blue
    z_value = ((0.0193339*red + 0.1191920*green +
                0.9503041*blue) / 1.08883)
    x_value = (x_value ** (1.0 / 3.0) if x_value > 0.00885645168
               else 7.787037*x_value + 16.0/116.0)
    y_value = (y_value ** (1.0 / 3.0) if y_value > 0.00885645168
               else 7.787037*y_value + 16.0/116.0)
    z_value = (z_value ** (1.0 / 3.0) if z_value > 0.00885645168
               else 7.787037*z_value + 16.0/116.0)
    return (116*y_value - 16,
            500*(x_value - y_value),
            200*(y_value - z_value))


def detect_color(frame, box, display_size):
    """Recognize the dominant LAB color in the central 60% of one box."""
    scale_x = RGB888P_SIZE[0] / display_size[0]
    scale_y = RGB888P_SIZE[1] / display_size[1]
    x0 = max(0, int((box[0] + box[2]*0.2) * scale_x))
    y0 = max(0, int((box[1] + box[3]*0.2) * scale_y))
    x1 = min(RGB888P_SIZE[0], int((box[0] + box[2]*0.8) * scale_x))
    y1 = min(RGB888P_SIZE[1], int((box[1] + box[3]*0.8) * scale_y))
    if x1 <= x0 or y1 <= y0:
        return "unknown"

    votes = [0] * len(COLOR_THRESHOLDS)
    total = 0
    grid = COLOR_SAMPLE_GRID
    for row in range(grid):
        y = min(y1 - 1, y0 + int((row + 0.5)*(y1-y0)/grid))
        for column in range(grid):
            x = min(x1 - 1,
                    x0 + int((column + 0.5)*(x1-x0)/grid))
            lab = rgb_to_lab(frame[0, y, x], frame[1, y, x],
                             frame[2, y, x])
            best_index = -1
            best_distance = 1e30
            for index, entry in enumerate(COLOR_THRESHOLDS):
                threshold = entry[1]
                distance = 0.0
                for axis in range(3):
                    low = threshold[axis*2]
                    high = threshold[axis*2 + 1]
                    if not low <= lab[axis] <= high:
                        break
                    distance += ((lab[axis] - (low+high)/2) /
                                 ((high-low)/2)) ** 2
                else:
                    if distance < best_distance:
                        best_index = index
                        best_distance = distance
            if best_index >= 0:
                votes[best_index] += 1
            total += 1

    winner = max(range(len(votes)), key=lambda index: votes[index])
    return (COLOR_THRESHOLDS[winner][0]
            if votes[winner] / total >= 0.2 else "unknown")


def frame_detections(result, colors, display_size):
    """Convert one YOLO result to class/u/v/confidence/color/box records."""
    detections = []
    if result is None or len(result) < 3:
        return detections
    boxes, class_ids, scores = result[0], result[1], result[2]
    scale_x = RGB888P_SIZE[0] / display_size[0]
    scale_y = RGB888P_SIZE[1] / display_size[1]
    for index in range(len(boxes)):
        class_id = int(class_ids[index])
        if not 0 <= class_id < len(LABELS):
            continue
        box = boxes[index]
        u = int(round((box[0] + box[2] / 2) * scale_x))
        v = int(round((box[1] + box[3] / 2) * scale_y))
        u = max(0, min(u, RGB888P_SIZE[0] - 1))
        v = max(0, min(v, RGB888P_SIZE[1] - 1))
        color_name = colors[index] if index < len(colors) else "unknown"
        camera_box = (
            max(0, int(round(box[0] * scale_x))),
            max(0, int(round(box[1] * scale_y))),
            max(1, int(round(box[2] * scale_x))),
            max(1, int(round(box[3] * scale_y))),
        )
        detections.append((class_id, u, v, float(scores[index]),
                           color_name, camera_box))
    return detections


def majority_color(samples):
    """Return the most frequent non-unknown color in one stable track."""
    counts = {}
    for sample in samples:
        color_name = sample[4]
        if color_name != "unknown":
            counts[color_name] = counts.get(color_name, 0) + 1
    best_name = "unknown"
    best_count = 0
    for color_name, count in counts.items():
        if count > best_count:
            best_name = color_name
            best_count = count
    return best_name


def stable_candidates(history):
    """Match same-class objects over five frames and return stable tracks."""
    if len(history) != STABLE_FRAMES:
        return []

    priority_ids = []
    for name in GRAB_PRIORITY:
        priority_ids.append(LABELS.index(name))
    tracks = []
    for detection in history[0]:
        if detection[0] in priority_ids:
            tracks.append([detection])

    max_distance_squared = TRACK_MATCH_DISTANCE_PX * TRACK_MATCH_DISTANCE_PX
    for detections in history[1:]:
        used = [False] * len(detections)
        matched_tracks = []
        for track in tracks:
            previous = track[-1]
            best_index = -1
            best_distance = max_distance_squared + 1
            for index in range(len(detections)):
                detection = detections[index]
                if used[index] or detection[0] != previous[0]:
                    continue
                du = detection[1] - previous[1]
                dv = detection[2] - previous[2]
                distance = du*du + dv*dv
                if distance < best_distance:
                    best_index = index
                    best_distance = distance
            if best_index >= 0:
                used[best_index] = True
                track.append(detections[best_index])
                matched_tracks.append(track)
        tracks = matched_tracks
        if not tracks:
            return []

    candidates = []
    middle = STABLE_FRAMES // 2
    for track in tracks:
        us = sorted(sample[1] for sample in track)
        vs = sorted(sample[2] for sample in track)
        if (us[-1] - us[0] > MAX_CENTER_SPREAD_PX or
                vs[-1] - vs[0] > MAX_CENTER_SPREAD_PX):
            continue
        confidence = sum(sample[3] for sample in track) / STABLE_FRAMES
        average_box = []
        for axis in range(4):
            average_box.append(int(round(
                sum(sample[5][axis] for sample in track) / STABLE_FRAMES
            )))
        candidates.append({
            "class_id": track[0][0],
            "class_name": LABELS[track[0][0]],
            "u": us[middle],
            "v": vs[middle],
            "confidence": confidence,
            "color": majority_color(track),
            "box": tuple(average_box),
        })
    return candidates


def candidate_counts(candidates):
    """Build an ordered class-count dictionary for grabbable objects."""
    counts = {}
    for class_name in GRAB_PRIORITY:
        counts[class_name] = 0
    for candidate in candidates:
        counts[candidate["class_name"]] += 1
    return counts


def same_object_counts(candidates):
    """Count stable objects with the same shape and recognized color."""
    counts = {}
    for candidate in candidates:
        if candidate["color"] == "unknown":
            continue
        key = (candidate["class_name"], candidate["color"])
        counts[key] = counts.get(key, 0) + 1
    return counts


def choose_grab_target(candidates):
    """Choose by class priority, same group, then class-specific distance."""
    group_counts = same_object_counts(candidates)

    # Class priority is strict: finish zhengfangti and yuanzhu before lengzhu.
    # Within the current highest-priority class, prefer a same shape/color
    # group. Single objects are used only when this class has no repeated group.
    for class_name in GRAB_PRIORITY:
        class_candidates = []
        repeated_candidates = []
        for candidate in candidates:
            if candidate["class_name"] != class_name:
                continue
            key = (candidate["class_name"], candidate["color"])
            same_count = group_counts.get(key, 1)
            candidate["same_count"] = same_count
            class_candidates.append(candidate)
            if same_count >= 2:
                repeated_candidates.append(candidate)

        if not class_candidates:
            continue
        selectable = (repeated_candidates if repeated_candidates
                      else class_candidates)
        selected = None
        selected_key = None

        for candidate in selectable:
            same_count = candidate["same_count"]
            pixel_distance = candidate["u"]**2 + candidate["v"]**2

            if class_name == "lengzhu":
                gripper = pixel_to_gripper((candidate["u"], candidate["v"]))
                if gripper is None:
                    continue
                mechanical_y = gripper[1]
                y_edge_distance = min(
                    abs(mechanical_y - GRIPPER_Y_MIN_CM),
                    abs(GRIPPER_Y_MAX_CM - mechanical_y),
                )
                candidate["gripper"] = gripper
                candidate["y_edge_distance"] = y_edge_distance
                # More identical objects first. For the same group size, grab
                # the lengzhu nearest to either mechanical Y boundary.
                sort_key = (-same_count, y_edge_distance, pixel_distance)
            else:
                sort_key = (-same_count, pixel_distance)

            if selected is None or sort_key < selected_key:
                selected = candidate
                selected_key = sort_key

        if selected is not None:
            return selected
    return None


def pixel_to_gripper(center):
    """Convert upright camera (u,v) to bounded gripper (X,Y) in cm."""
    u, v = float(center[0]), float(center[1])
    transform = PIXEL_TO_GRIPPER
    denominator = transform[6] * u + transform[7] * v + transform[8]
    if abs(denominator) < 1e-9:
        return None
    x = ((transform[0] * u + transform[1] * v + transform[2]) /
         denominator + GRIPPER_X_OFFSET_MM / 10.0)
    y = ((transform[3] * u + transform[4] * v + transform[5]) /
         denominator + GRIPPER_y_OFFSET_MM / 10.0)
    # if x > GRIPPER_X_HIGH_COMPENSATION_CM:
    #     x += GRIPPER_X_OUTER_OFFSET_MM / 10.0
    # elif x < GRIPPER_X_LOW_COMPENSATION_CM:
    #     x -= GRIPPER_X_OUTER_OFFSET_MM0 / 10.0
    x = max(GRIPPER_X_MIN_CM, min(x, GRIPPER_X_MAX_CM))
    y = max(GRIPPER_Y_MIN_CM, min(y, GRIPPER_Y_MAX_CM))
    return x, y


def main():
    camera = HeadlessCamera(RGB888P_SIZE)
    detector = None
    uart = None

    try:
        camera.create()
        # Headless YOLO coordinates map directly to the 640x480 AI frame.
        display_size = RGB888P_SIZE

        detector = YOLO26(
            task_type="detect",
            mode="video",
            kmodel_path=DETECT_KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONFIDENCE_THRESHOLD,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=0
        )
        detector.config_preprocess()
        uart = YbUart(baudrate=UART_BAUDRATE)

        print("K230 headless YOLO26 detector started")
        print("DETECT_MODEL =", DETECT_KMODEL_PATH)
        print("DISPLAY = disabled")
        print("AI_FRAME_SIZE =", RGB888P_SIZE)
        print("GRAB_PRIORITY =", GRAB_PRIORITY)
        print("Waiting for lowercase s from controller")
        frame_count = 0
        result = [[], [], []]
        colors = []
        history = []
        recognizing = False
        selected = None

        while True:
            with ScopedTiming("total", TIMING_DEBUG):
                image = camera.get_frame()
                data = uart.read()
                if isinstance(data, str):
                    data = data.encode()
                if not recognizing and data and START_COMMAND in data:
                    recognizing = True
                    history = []
                    selected = None
                    print("Recognition cycle started")

                if (recognizing and
                        frame_count >= CAMERA_WARMUP_FRAMES and
                        frame_count % INFERENCE_INTERVAL_FRAMES == 0):
                    result = [[], [], []]
                    colors = []
                    result = detector.run(image)
                    if result is None or len(result) < 3:
                        result = [[], [], []]
                    colors = [detect_color(image, box, display_size)
                              for box in result[0]]
                    detections = frame_detections(
                        result, colors, display_size
                    )
                    history.append(detections)
                    if len(history) > STABLE_FRAMES:
                        history.pop(0)

                    candidates = stable_candidates(history)
                    if candidates:
                        counts = candidate_counts(candidates)
                        selected = choose_grab_target(candidates)
                        if selected is None:
                            gc.collect()
                            continue
                        print("GRAB_COUNTS:", counts)
                        print("SELECTED:%s,PIXEL:%d,%d,COLOR:%s,SAME:%d,CONF:%.3f" % (
                            selected["class_name"], selected["u"],
                            selected["v"], selected["color"],
                            selected["same_count"],
                            selected["confidence"]
                        ))
                        final_name = selected["class_name"]
                        gripper = selected.get("gripper")
                        if gripper is None:
                            gripper = pixel_to_gripper(
                                (selected["u"], selected["v"])
                            )
                        if (selected["class_name"] == "lengzhu" and
                                gripper is not None):
                            print("LENGZHU:Y=%.2f,EDGE_DISTANCE=%.2f" % (
                                gripper[1], selected["y_edge_distance"]
                            ))
                        # Discard commands accumulated while recognizing. A
                        # completed grab requires a fresh s for the next cycle.
                        uart.read()
                        if gripper is None:
                            uart.send(b"TARGET,INVALID\r\n")
                            print("TARGET,INVALID")
                        else:
                            target_message = (
                                "TARGET,1,%s,%s,%.2f,%.2f,CM" % (
                                    final_name,
                                    selected["color"],
                                    gripper[0], gripper[1]
                                )
                            )
                            uart.send((target_message + "\r\n").encode())
                            print("UART:", target_message)
                        recognizing = False
                        history = []
                        print("Result sent; waiting for next s")
                frame_count += 1
                if frame_count % GC_INTERVAL_FRAMES == 0:
                    gc.collect()

    except KeyboardInterrupt:
        print("K230 object detector stopped")
    except BaseException as error:
        sys.print_exception(error)
    finally:
        if detector is not None:
            detector.deinit()
        if camera is not None:
            camera.destroy()
        if uart is not None:
            uart.deinit()
        gc.collect()


if __name__ == "__main__":
    main()
