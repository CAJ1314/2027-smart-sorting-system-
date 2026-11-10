from libs.YOLO import YOLO11
from libs.Utils import ScopedTiming
from ybUtils.YbUart import YbUart
from media.sensor import *
from media.media import MediaManager
import gc
import sys
import time


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


def point_in_curved_roi(point_x, point_y, roi):
    """Keep points inside the arc, left edge, and bottom edge."""
    x, y, width, height = roi
    if not (x <= point_x <= x + width and
            y <= point_y <= y + height):
        return False
    normalized_x = (point_x - x) / width
    normalized_y = (point_y - y) / height
    return ((1.0 - normalized_x) ** 0.5 +
            normalized_y ** 0.5 >= 1.0)


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
COLOR_SAMPLE_GRID = 12

def rgb_to_lab(red, green, blue):
    # sRGB -> CIE LAB, D65 white; L: 0..100, a/b signed.
    channels = []
    for value in (red, green, blue):
        value = float(value) / 255.0
        channels.append(value / 12.92 if value <= 0.04045
                        else ((value + 0.055) / 1.055) ** 2.4)
    r, g, b = channels
    xyz = ((0.4124564*r + 0.3575761*g + 0.1804375*b) / 0.95047,
           0.2126729*r + 0.7151522*g + 0.0721750*b,
           (0.0193339*r + 0.1191920*g + 0.9503041*b) / 1.08883)
    values = [v ** (1.0 / 3.0) if v > 0.00885645168
              else 7.787037*v + 16.0/116.0 for v in xyz]
    return (116*values[1] - 16, 500*(values[0] - values[1]),
            200*(values[1] - values[2]))


def detect_color(frame, box, camera_size, display_size):
    # Sensor AI frame is RGB888 planar [3, height, width].
    # YOLO video boxes are display coordinates. Sample the central 60%
    # of the box in camera coordinates to reduce background contamination.
    sx = camera_size[0] / display_size[0]
    sy = camera_size[1] / display_size[1]
    x0 = max(0, int((box[0] + box[2]*0.2) * sx))
    y0 = max(0, int((box[1] + box[3]*0.2) * sy))
    x1 = min(camera_size[0], int((box[0] + box[2]*0.8) * sx))
    y1 = min(camera_size[1], int((box[1] + box[3]*0.8) * sy))
    if x1 <= x0 or y1 <= y0:
        return "unknown", 0.0
    votes = [0] * len(COLOR_THRESHOLDS)
    total = 0
    grid = COLOR_SAMPLE_GRID
    for row in range(grid):
        y = min(y1 - 1, y0 + int((row + 0.5)*(y1-y0)/grid))
        for col in range(grid):
            x = min(x1 - 1, x0 + int((col + 0.5)*(x1-x0)/grid))
            lab = rgb_to_lab(frame[0, y, x], frame[1, y, x],
                             frame[2, y, x])
            best_index, best_distance = -1, 1e30
            for index, entry in enumerate(COLOR_THRESHOLDS):
                threshold = entry[1]
                distance = 0.0
                for axis in range(3):
                    low, high = threshold[2*axis:2*axis+2]
                    if not low <= lab[axis] <= high:
                        break
                    distance += ((lab[axis] - (low+high)/2) /
                                 ((high-low)/2)) ** 2
                else:
                    if distance < best_distance:
                        best_index, best_distance = index, distance
            if best_index >= 0:
                votes[best_index] += 1
            total += 1
    winner = max(range(len(votes)), key=lambda index: votes[index])
    ratio = votes[winner] / total
    # This is a sample vote ratio, not a model confidence.
    return (COLOR_THRESHOLDS[winner][0] if ratio >= 0.2 else "unknown",
            ratio)


if __name__ == "__main__":
    # Use a YOLO11 detection kmodel with the labels and input size below.
    kmodel_path = "/sdcard/best_noclip_uint80.kmodel"
    # The order must exactly match dataset.yaml and model export settings.
    labels = [
        "zhengfangti",
        "liulengzhu",
        "sanlengzhu",
        "qiu",
        "wulengzhu",
        "yuanzhu",
        "changfangti",

    ]
    model_input_size = [640, 640]
    confidence_threshold = 0.4
    nms_threshold = 0.40
    # [x, y, width, height] in the 640x360 AI frame.
    # Converted from the former 640x480 ROI by multiplying Y and height by
    # 360 / 480. Fine-tune these values if the physical field of view changes.
    roi = [232, 114, 174, 151]
    uart_baudrate = 115200
    start_command = "s"
    # Wait long enough for exposure/color to settle, then require the same
    # shape and a known color across several consecutive frames.
    minimum_recognition_ms = 1000
    confirmation_frames = 5
    detection_timeout_ms = 3000

    # AI capture only. Display is intentionally not initialized.
    rgb888p_size = [640, 360]

    # Serial output and full garbage collection are relatively expensive on
    # K230. Run them periodically instead of blocking every video frame.
    maintenance_interval = 30

    camera = HeadlessCamera(rgb888p_size)
    yolo = None
    uart = None

    try:
        camera.create()
        # In headless mode YOLO output coordinates use the AI frame itself.
        display_size = rgb888p_size

        yolo = YOLO11(
            task_type="detect",
            mode="video",
            kmodel_path=kmodel_path,
            labels=labels,
            rgb888p_size=rgb888p_size,
            model_input_size=model_input_size,
            display_size=display_size,
            conf_thresh=confidence_threshold,
            nms_thresh=nms_threshold,
            max_boxes_num=20,
            debug_mode=0
        )
        yolo.config_preprocess()

        uart = YbUart(baudrate=uart_baudrate)

        def send_uart(message):
            print("[UART] Send:", message)
            uart.send((message + "\r\n").encode())

        def get_start_command(receive_buffer):
            data = uart.read()
            if data:
                if isinstance(data, bytes):
                    try:
                        data = data.decode()
                    except BaseException:
                        data = str(data)
                receive_buffer += str(data)

            if start_command in receive_buffer:
                return True, ""

            # UART reads may contain a partial command. Keep only the suffix
            # which could still become the start command on the next read.
            max_prefix_length = min(len(receive_buffer), len(start_command))
            for length in range(max_prefix_length, 0, -1):
                if receive_buffer.endswith(start_command[:length]):
                    return False, receive_buffer[-length:]
            return False, ""

        print("Block sorter ready; waiting for:", start_command)
        send_uart("K230_START\r\n")
        frame_count = 0
        recognizing = False
        recognition_start_ms = 0
        last_class_id = -1
        last_color_name = "unknown"
        matching_frames = 0
        receive_buffer = ""
        while True:
            with ScopedTiming("total", 0):
                img = camera.get_frame()
                started, receive_buffer = get_start_command(receive_buffer)
                if not recognizing and started:
                    recognizing = True
                    recognition_start_ms = time.ticks_ms()
                    last_class_id = -1
                    last_color_name = "unknown"
                    matching_frames = 0
                    print("Recognition started")
                    send_uart("K230,RECOGNIZING")

                # Waiting mode does not run YOLO. A new command starts one
                # recognition cycle for one incoming block.
                res = [[], [], []]
                colors = []
                if recognizing:
                    res = yolo.run(img)
                    frame_count += 1

                    boxes = []
                    class_ids = []
                    scores = []
                    if res is not None and len(res) >= 3:
                        # Keep only targets whose center lies within the ROI.
                        for index in range(len(res[0])):
                            box = res[0][index]
                            center_x = (box[0] + box[2] / 2) * rgb888p_size[0] / display_size[0]
                            center_y = (box[1] + box[3] / 2) * rgb888p_size[1] / display_size[1]
                            if point_in_curved_roi(center_x, center_y, roi):
                                boxes.append(box)
                                class_ids.append(res[1][index])
                                scores.append(res[2][index])
                    res = [boxes, class_ids, scores]
                    colors = [detect_color(img, box, rgb888p_size, display_size)
                              for box in boxes]

                    if boxes:
                        # YOLO11 returns detections sorted by confidence.
                        class_id = int(class_ids[0])
                        confidence = float(scores[0])
                        color_name = colors[0][0] if colors else "unknown"
                        if (class_id == last_class_id and
                                color_name == last_color_name and
                                color_name != "unknown"):
                            matching_frames += 1
                        else:
                            last_class_id = class_id
                            last_color_name = color_name
                            matching_frames = (1 if color_name != "unknown"
                                               else 0)

                        elapsed_ms = time.ticks_diff(
                            time.ticks_ms(), recognition_start_ms
                        )
                        if (elapsed_ms >= minimum_recognition_ms and
                                matching_frames >= confirmation_frames and
                                0 <= class_id < len(labels)):
                            class_name = labels[class_id]
                            message = "SHAPE:{},COLOR:{}".format(
                                class_name, color_name
                            )
                            color_ratio = colors[0][1] if colors else 0.0
                            print("Recognition complete:", message,
                                  "color vote ratio:", round(color_ratio, 3))
                            send_uart(message)
                            recognizing = False
                            matching_frames = 0
                            send_uart("K230,READY")
                    else:
                        last_class_id = -1
                        last_color_name = "unknown"
                        matching_frames = 0

                    if (recognizing and time.ticks_diff(
                            time.ticks_ms(), recognition_start_ms
                    ) >= detection_timeout_ms):
                        print("Recognition timed out")
                        send_uart("none")
                        recognizing = False
                        send_uart("K230,READY")

                if frame_count >= maintenance_interval:
                    gc.collect()
                    frame_count = 0

    except KeyboardInterrupt:
        print("Block video detection stopped")
    except BaseException as error:
        sys.print_exception(error)
    finally:
        if yolo is not None:
            yolo.deinit()
        if camera is not None:
            camera.destroy()
        if uart is not None:
            uart.deinit()
        gc.collect()
