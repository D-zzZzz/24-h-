import time

from media.sensor import *
from media.display import *
from media.media import *

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart


# 灰度二值化 + 八邻域连续跟踪 + 侧边丢线 + 二次贝塞尔中线。
IMG_W = 192
IMG_H = 120
IMG_CENTER_X = IMG_W // 2

# Otsu阈值限幅，防止曝光异常时阈值完全跳变。
THRESHOLD_MIN = 20
THRESHOLD_MAX = 90

# 从底部向上寻找黑线起点。
START_SEARCH_ROWS = 25
BOTTOM_MARGIN = 1

# 黑线起始宽度限制。
MIN_LINE_WIDTH = 2
MAX_START_LINE_WIDTH = 66

# 至少跟踪若干行才认为结果有效。
MIN_TRACK_ROWS = 3

# 对应 Chezi 的 Deal_Left/Right/Bottom/Top，坐标 y 从底部向上递增。
DEAL_LEFT = 12
DEAL_RIGHT = IMG_W - 13
DEAL_BOTTOM = 5
DEAL_TOP = IMG_H - 13
SIDE_LOST_MIN_ROWS = 2
CURVE_CURVATURE = 55
LOOKAHEAD_Y = DEAL_BOTTOM + 30

# 普通弯状态跨帧保持；仅底部、顶部出口同时恢复时退出。
NORMAL_CURVE_IDLE = 0
NORMAL_CURVE_LEFT = 1
NORMAL_CURVE_RIGHT = 2
normal_curve_state = NORMAL_CURVE_IDLE
normal_curve_end_y = DEAL_BOTTOM + 45

# 二值图中黑线为255，白底为0。
LINE_PIXEL = 128

# 协议不能直接传负数时使用偏置编码。
ERROR_BIAS = 1000

# 正常离线运行不输出逐帧 USB 日志；调试帧率时可打开。
DEBUG_LOG = False


uart = YbUart(baudrate=115200)
pto = YbProtocol()


def init_sensor():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    return sensor


def init_display():
    Display.init(Display.ST7701, to_ide=False)
    MediaManager.init()


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def grayscale_binary(img):
    """灰度图执行Otsu二值化，暗色黑线变为255，白底变为0。"""
    histogram = img.get_histogram()
    threshold = histogram.get_threshold().value()
    threshold = clamp(threshold, THRESHOLD_MIN, THRESHOLD_MAX)
    img.binary([(0, threshold)])
    return threshold


def is_line_pixel(img, x, y):
    return img.get_pixel(x, y) >= LINE_PIXEL


def get_runs_on_row(img, y):
    """得到一行中的黑线连续区间。"""
    runs = []
    x = 1
    while x < IMG_W - 1:
        if not is_line_pixel(img, x, y):
            x += 1
            continue

        left = x
        while x < IMG_W - 1 and is_line_pixel(img, x, y):
            x += 1
        runs.append((left, x - 1))

    return runs


def find_start_run(img):
    """从画面底部向上寻找黑线起点。"""
    first_y = IMG_H - 1 - BOTTOM_MARGIN
    last_y = max(1, first_y - START_SEARCH_ROWS)

    for actual_y in range(first_y, last_y - 1, -1):
        best_run = None
        best_distance = IMG_W + 1

        for left, right in get_runs_on_row(img, actual_y):
            width = right - left + 1
            if width < MIN_LINE_WIDTH or width > MAX_START_LINE_WIDTH:
                continue

            center = (left + right) // 2
            distance = abs(center - IMG_CENTER_X)
            if distance < best_distance:
                best_distance = distance
                best_run = (left, right, actual_y)

        if best_run is not None:
            return best_run

    return None


def expand_run_from_seed(img, seed_x, actual_y):
    """从黑线内部一点向左右扩展，得到这一行左右边线。"""
    left = seed_x
    right = seed_x

    while left > 1 and is_line_pixel(img, left - 1, actual_y):
        left -= 1
    while right < IMG_W - 2 and is_line_pixel(img, right + 1, actual_y):
        right += 1

    return left, right


def find_8connected_next_run(img, actual_y, previous_left, previous_right):
    """在上一行左右边线的八邻域范围内寻找下一行黑线。"""
    search_left = max(1, previous_left - 1)
    search_right = min(IMG_W - 2, previous_right + 1)
    previous_center = (previous_left + previous_right) // 2

    best_run = None
    best_distance = IMG_W + 1
    x = search_left

    while x <= search_right:
        if not is_line_pixel(img, x, actual_y):
            x += 1
            continue

        left, right = expand_run_from_seed(img, x, actual_y)
        center = (left + right) // 2
        distance = abs(center - previous_center)

        if distance < best_distance:
            best_distance = distance
            best_run = (left, right)

        x = max(x + 1, right + 1)

    return best_run


def get_side_lost_centers(left_line, right_line, side_x, start_y, end_y):
    """收集已跟踪赛道在侧边连续出界段的中心 y，最多保留两段。"""
    centers = []
    run_start = -1
    for logical_y in range(max(start_y, DEAL_BOTTOM), end_y + 2):
        touches_side = (
            logical_y <= end_y
            and 0 <= left_line[logical_y] <= side_x <= right_line[logical_y]
        )
        if touches_side:
            if run_start < 0:
                run_start = logical_y
        elif run_start >= 0:
            run_end = logical_y - 1
            # 与 C 一样，排除贴着底部入口的侧边连通段；短孤点不作出口。
            if (run_start > DEAL_BOTTOM + 2
                    and run_end - run_start + 1 >= SIDE_LOST_MIN_ROWS):
                centers.append((run_start + run_end) // 2)
                if len(centers) == 2:
                    break
            run_start = -1
    return centers


def get_curve_line(mid_line, x1, y1, x2, y2, curvature_n):
    """对应 C 的 Get_curve_line：逐行生成二次贝塞尔中线。"""
    if y1 < 0 or y1 >= y2 or y2 >= IMG_H:
        return False
    control_x = (x1 + x2) / 2.0 + curvature_n * 0.5
    for logical_y in range(y1, y2 + 1):
        t = (logical_y - y1) / (y2 - y1)
        x = ((1.0 - t) * (1.0 - t) * x1
             + 2.0 * (1.0 - t) * t * control_x
             + t * t * x2)
        mid_line[logical_y] = int(clamp(x, 0, IMG_W - 1))
    return True


def trace_line_8neighbor(img, start_run):
    """
    从底部起点向上进行八邻域连续生长。
    logical_y=0为图像底部，logical_y向上递增。
    """
    global normal_curve_state, normal_curve_end_y
    left_line = [-1] * IMG_H
    right_line = [-1] * IMG_H
    mid_line = [-1] * IMG_H

    previous_left, previous_right, start_actual_y = start_run
    start_logical_y = IMG_H - 1 - start_actual_y

    left_line[start_logical_y] = previous_left
    right_line[start_logical_y] = previous_right
    mid_line[start_logical_y] = (previous_left + previous_right) // 2

    tracked_rows = 1
    lost_y = start_logical_y + 1

    for logical_y in range(start_logical_y + 1, DEAL_TOP + 1):
        actual_y = IMG_H - 1 - logical_y
        next_run = find_8connected_next_run(
            img, actual_y, previous_left, previous_right
        )

        if next_run is None:
            lost_y = logical_y
            break

        previous_left, previous_right = next_run
        left_line[logical_y] = previous_left
        right_line[logical_y] = previous_right
        mid_line[logical_y] = (previous_left + previous_right) // 2
        tracked_rows += 1
        lost_y = logical_y + 1

    # C 的 l_center/r_center 是侧边出界段中心，不是向上搜索停止的行。
    end_y = min(lost_y - 1, DEAL_TOP)
    left_lost = get_side_lost_centers(
        left_line, right_line, DEAL_LEFT, start_logical_y, end_y
    )
    right_lost = get_side_lost_centers(
        left_line, right_line, DEAL_RIGHT, start_logical_y, end_y
    )
    bottom_open = left_line[DEAL_BOTTOM] >= 0
    top_open = left_line[DEAL_TOP] >= 0

    if normal_curve_state != NORMAL_CURVE_IDLE and bottom_open and top_open:
        normal_curve_state = NORMAL_CURVE_IDLE
        normal_curve_end_y = DEAL_BOTTOM + 45

    if normal_curve_state == NORMAL_CURVE_IDLE:
        if not top_open and left_lost and right_lost:
            # 尚未锁定方向的双侧出口仍交给无效标志处理。
            return left_line, right_line, [-1] * IMG_H, 0, tracked_rows
        if bottom_open and not top_open:
            if left_lost and not right_lost:
                normal_curve_state = NORMAL_CURVE_LEFT
            elif right_lost and not left_lost:
                normal_curve_state = NORMAL_CURVE_RIGHT

    if normal_curve_state != NORMAL_CURVE_IDLE:
        if normal_curve_state == NORMAL_CURVE_LEFT:
            side_lost = left_lost
            end_x = DEAL_LEFT
            curvature_n = CURVE_CURVATURE
        else:
            side_lost = right_lost
            end_x = DEAL_RIGHT
            curvature_n = -CURVE_CURVATURE
        # 暂时没有同侧出口时保留弯心；太近的弯心不能截断固定前瞻行。
        if side_lost and side_lost[-1] >= LOOKAHEAD_Y:
            normal_curve_end_y = side_lost[-1]
        lost_y = min(DEAL_TOP, max(normal_curve_end_y,
                                  LOOKAHEAD_Y, start_logical_y + 1))
        mid_line = [-1] * IMG_H
        get_curve_line(mid_line, IMG_CENTER_X, start_logical_y,
                       end_x, lost_y, curvature_n)

    return left_line, right_line, mid_line, lost_y, tracked_rows



def calculate_error_at_lookahead(mid_line):
    """固定取 DEAL_BOTTOM+30 行的中线点计算误差。"""
    target_y = LOOKAHEAD_Y
    if target_y < 0 or target_y >= IMG_H:
        return 0, 0, 0, 0

    target_x = mid_line[target_y]
    if target_x < 0:
        return 0, 0, target_y, 0

    error = target_x - IMG_CENTER_X
    return error, target_x, target_y, 1


def send_tracking_data(error, target_x, target_y, valid):
    """小图处理，发送仍使用原 640x480 坐标以兼容 STM32。"""
    send_error = int(error * 640 / IMG_W)
    send_x = send_error + 320
    send_y = int(target_y * 480 / IMG_H)
    packet = pto.get_color_data(
        send_error + ERROR_BIAS,
        send_x,
        send_y,
        int(valid)
    )
    uart.send(packet)


def draw_tracking_result(img, left_line, right_line, mid_line,
                         target_x, target_y, error, threshold, valid):
    """在二值图上绘制左右边线、中线和误差点。"""
    if mid_line is not None:
        previous_mid = None
        for logical_y in range(0, IMG_H, 6):
            actual_y = IMG_H - 1 - logical_y

            if left_line[logical_y] >= 0:
                img.draw_cross(left_line[logical_y], actual_y,
                               color=128, size=1, thickness=1)
            if right_line[logical_y] >= 0:
                img.draw_cross(right_line[logical_y], actual_y,
                               color=128, size=1, thickness=1)
            if mid_line[logical_y] >= 0:
                point = (mid_line[logical_y], actual_y)
                if previous_mid is not None:
                    img.draw_line((previous_mid[0], previous_mid[1],
                                   point[0], point[1]), color=128, thickness=1)
                img.draw_cross(point[0], point[1], color=128, size=1, thickness=1)
                previous_mid = point
            else:
                previous_mid = None

    img.draw_line(
        (IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1),
        color=128, thickness=1
    )

    if valid:
        target_actual_y = IMG_H - 1 - target_y
        img.draw_cross(target_x, target_actual_y,
                       color=128, size=4, thickness=1)
        text = "ERR:%d X:%d Y:%d T:%d" % (
            error, target_x, target_y, threshold
        )
    else:
        text = "LINE LOST T:%d" % threshold

    img.draw_string_advanced(0, 0, 12, text, color=128)
    img.draw_string_advanced(0, 14, 12,
                            "TURN:%s" % ("IDLE", "LEFT", "RIGHT")[normal_curve_state],
                            color=128)


def main():
    sensor = None
    sensor_started = False
    display_initialized = False
    media_initialized = False

    try:
        sensor = init_sensor()
        Display.init(Display.ST7701, to_ide=False)
        display_initialized = True
        MediaManager.init()
        media_initialized = True
        sensor.run()
        sensor_started = True
        time.sleep_ms(500)

        clock = time.clock()

        while True:
            clock.tick()
            img = sensor.snapshot()

            # 1. 灰度二值化。
            threshold = grayscale_binary(img)

            # 2. 从底部寻找黑线起点。
            start_run = find_start_run(img)

            left_line = None
            right_line = None
            mid_line = None
            error = 0
            target_x = 0
            target_y = 0
            valid = 0

            if start_run is not None:
                # 3. 普通弯状态保持方向，底部和顶部出口恢复后才退出。
                left_line, right_line, mid_line, lost_y, tracked_rows = \
                    trace_line_8neighbor(img, start_run)

                # 4. 固定取 DEAL_BOTTOM+30 行的中线 x，减去图像中心。
                error, target_x, target_y, point_valid = \
                    calculate_error_at_lookahead(mid_line)

                if point_valid and tracked_rows >= MIN_TRACK_ROWS:
                    valid = 1
            elif normal_curve_state != NORMAL_CURVE_IDLE:
                # 已锁定弯道时，即使无起点也从底部中心补曲线，继续发送误差。
                left_line = [-1] * IMG_H
                right_line = [-1] * IMG_H
                mid_line = [-1] * IMG_H
                if normal_curve_state == NORMAL_CURVE_LEFT:
                    end_x = DEAL_LEFT
                    curvature_n = CURVE_CURVATURE
                else:
                    end_x = DEAL_RIGHT
                    curvature_n = -CURVE_CURVATURE
                end_y = min(DEAL_TOP, max(normal_curve_end_y,
                                         LOOKAHEAD_Y, DEAL_BOTTOM + 1))
                get_curve_line(mid_line, IMG_CENTER_X, DEAL_BOTTOM,
                               end_x, end_y, curvature_n)
                error, target_x, target_y, point_valid = \
                    calculate_error_at_lookahead(mid_line)
                valid = point_valid

            # 5. 每帧发送给STM32。
            send_tracking_data(error, target_x, target_y, valid)

            fps = clock.fps()
            draw_tracking_result(
                img, left_line, right_line, mid_line,
                target_x, target_y, error, threshold, valid
            )
            Display.show_image(img)

            if DEBUG_LOG:
                print(
                    "valid=%d threshold=%d target_y=%d target_x=%d error=%d fps=%.2f"
                    % (valid, threshold, target_y, target_x, error, fps)
                )

    except KeyboardInterrupt as e:
        print("User interrupted:", e)
    except Exception as e:
        print("Error occurred:", e)
    finally:
        if sensor is not None and sensor_started:
            sensor.stop()
        if display_initialized:
            Display.deinit()
        if media_initialized:
            MediaManager.deinit()


if __name__ == "__main__":
    main()
