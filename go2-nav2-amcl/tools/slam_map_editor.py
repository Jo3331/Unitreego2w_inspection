#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS SLAM 2D 地图手动编辑器（PGM + YAML 自动配套版）
================================================

用途：
    手动编辑 ROS / Nav2 使用的 2D Occupancy Grid 地图。

地图像素：
    0   = 黑色 = Occupied / Wall
    254 = 白色 = Free
    205 = 灰色 = Unknown

【运行】
    python3 slam_map_editor.py map4.pgm

指定输出名：
    python3 slam_map_editor.py map4.pgm -o map4_clean.pgm

默认输出：
    map4_edited.pgm
    map4_edited.yaml

如果输入目录存在同名 YAML：
    map4.pgm
    map4.yaml

保存时程序会自动复制 map4.yaml 的参数，并把：
    image: map4.pgm
改成：
    image: map4_edited.pgm

resolution、origin、negate、occupied_thresh、free_thresh 等保持不变。

如果找不到 map4.yaml：
    PGM 仍然正常保存；
    但程序不会猜测 resolution/origin，也不会凭空创建错误 YAML。

【操作】
    1 = Wall / Occupied（黑色）
    2 = Free（白色）
    3 = Unknown（灰色）

    鼠标左键拖动       = 画笔
    Shift + 左键拖动  = 矩形区域填充
    + / =             = 增大画笔
    -                 = 减小画笔
    U                 = 撤销
    S                 = 保存 PGM + 配套 YAML
    Q / ESC           = 退出

【使用建议】
    已确认现场为空的灰色盲区 -> 2 Free
    错误黑色 LiDAR 噪点      -> 2 Free
    确认存在墙/固定障碍物     -> 1 Wall
    无法确认的区域            -> 3 Unknown

注意：
    本工具不裁剪、不缩放地图，因此原 YAML 的 resolution 和 origin
    可以继续使用。若未来增加裁剪/旋转功能，则需要重新计算地图参数。
"""

import argparse
import re
import shutil
from pathlib import Path

import cv2

OCCUPIED = 0
FREE = 254
UNKNOWN = 205

img = None
history = []
drawing = False
mode = "free"
brush_size = 8

rect_selecting = False
rect_start = None
rect_end = None


def push_history():
    history.append(img.copy())
    if len(history) > 40:
        history.pop(0)


def current_value():
    return {
        "wall": OCCUPIED,
        "free": FREE,
        "unknown": UNKNOWN
    }[mode]


def paint(x, y):
    cv2.circle(img, (x, y), brush_size, current_value(), -1)


def mouse_callback(event, x, y, flags, param):
    global drawing, rect_selecting, rect_start, rect_end

    shift = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)

    if event == cv2.EVENT_LBUTTONDOWN:
        push_history()

        if shift:
            rect_selecting = True
            rect_start = (x, y)
            rect_end = (x, y)
        else:
            drawing = True
            paint(x, y)

    elif event == cv2.EVENT_MOUSEMOVE:
        if rect_selecting:
            rect_end = (x, y)
        elif drawing:
            paint(x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        if rect_selecting:
            rect_end = (x, y)
            cv2.rectangle(
                img,
                rect_start,
                rect_end,
                current_value(),
                -1
            )
            rect_selecting = False

        elif drawing:
            paint(x, y)
            drawing = False


def find_source_yaml(pgm_path):
    """优先寻找与输入 PGM 同名的 .yaml / .yml。"""
    yaml_path = pgm_path.with_suffix(".yaml")
    if yaml_path.exists():
        return yaml_path

    yml_path = pgm_path.with_suffix(".yml")
    if yml_path.exists():
        return yml_path

    return None


def create_output_yaml(source_yaml, output_pgm):
    """
    复制原 YAML，并仅修改 image 字段。
    不依赖 PyYAML，避免重排用户原 YAML 的格式/注释。
    """
    if source_yaml is None:
        print("提示：未找到输入地图对应的 YAML。")
        print("      已保存 PGM，但没有生成 YAML。")
        print("      请提供原 YAML 后再配套使用。")
        return None

    output_yaml = output_pgm.with_suffix(".yaml")

    text = source_yaml.read_text(encoding="utf-8")

    # YAML 常见写法：
    # image: map4.pgm
    # image: "map4.pgm"
    # image: 'map4.pgm'
    pattern = r'(?m)^(\s*image\s*:\s*)(["\']?)(.*?)(\2)\s*$'

    replacement_done = False

    def replace_image(match):
        nonlocal replacement_done
        replacement_done = True
        prefix = match.group(1)
        quote = match.group(2)
        return f"{prefix}{quote}{output_pgm.name}{quote}"

    new_text = re.sub(pattern, replace_image, text, count=1)

    if not replacement_done:
        # 极少数 YAML 没有 image 字段时，不猜测其它参数，只补 image。
        new_text = f"image: {output_pgm.name}\n" + text

    output_yaml.write_text(new_text, encoding="utf-8")

    print("YAML 已生成：", output_yaml)
    print("YAML 来源：", source_yaml)
    print("image 已更新为：", output_pgm.name)

    return output_yaml


def save_map(output_pgm, source_yaml):
    ok = cv2.imwrite(str(output_pgm), img)

    if not ok:
        print("错误：PGM 保存失败：", output_pgm)
        return

    print("PGM 已保存：", output_pgm)

    create_output_yaml(
        source_yaml,
        output_pgm
    )


def main():
    global img, mode, brush_size, history

    parser = argparse.ArgumentParser(
        description="Manual ROS SLAM PGM map editor with automatic YAML copy/update."
    )

    parser.add_argument(
        "map",
        help="输入 .pgm 地图"
    )

    parser.add_argument(
        "-o",
        "--output",
        help="输出 .pgm 文件名"
    )

    args = parser.parse_args()

    input_pgm = Path(args.map)

    if args.output:
        output_pgm = Path(args.output)
    else:
        output_pgm = input_pgm.with_name(
            input_pgm.stem + "_edited.pgm"
        )

    img = cv2.imread(
        str(input_pgm),
        cv2.IMREAD_GRAYSCALE
    )

    if img is None:
        raise FileNotFoundError(
            f"无法读取地图：{input_pgm}"
        )

    source_yaml = find_source_yaml(input_pgm)

    print("\n======================================")
    print("ROS SLAM Map Editor")
    print("======================================")
    print("输入 PGM :", input_pgm)

    if source_yaml:
        print("输入 YAML:", source_yaml)
    else:
        print("输入 YAML: 未找到")

    print("输出 PGM :", output_pgm)
    print("输出 YAML:", output_pgm.with_suffix(".yaml"))
    print("--------------------------------------")
    print("1 = WALL    黑色")
    print("2 = FREE    白色")
    print("3 = UNKNOWN 灰色")
    print("左键 = 涂抹")
    print("Shift + 左键拖动 = 矩形填充")
    print("+/- = 调整画笔")
    print("U = 撤销")
    print("S = 保存 PGM + YAML")
    print("Q = 退出")
    print("======================================\n")

    window = "ROS SLAM Map Editor"

    cv2.namedWindow(
        window,
        cv2.WINDOW_NORMAL
    )

    cv2.setMouseCallback(
        window,
        mouse_callback
    )

    while True:
        preview = cv2.cvtColor(
            img,
            cv2.COLOR_GRAY2BGR
        )

        if rect_selecting and rect_start and rect_end:
            cv2.rectangle(
                preview,
                rect_start,
                rect_end,
                (0, 0, 255),
                1
            )

        cv2.putText(
            preview,
            f"Mode: {mode.upper()} | Brush: {brush_size}px",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            1,
            cv2.LINE_AA
        )

        cv2.imshow(
            window,
            preview
        )

        key = cv2.waitKey(10) & 0xFF

        if key == ord("1"):
            mode = "wall"
            print("模式 -> WALL")

        elif key == ord("2"):
            mode = "free"
            print("模式 -> FREE")

        elif key == ord("3"):
            mode = "unknown"
            print("模式 -> UNKNOWN")

        elif key in (ord("+"), ord("=")):
            brush_size += 2
            print("画笔：", brush_size)

        elif key == ord("-"):
            brush_size = max(
                1,
                brush_size - 2
            )
            print("画笔：", brush_size)

        elif key in (ord("u"), ord("U")):
            if history:
                img = history.pop()
                print("已撤销")
            else:
                print("没有可撤销的操作")

        elif key in (ord("s"), ord("S")):
            save_map(
                output_pgm,
                source_yaml
            )

        elif key in (
            ord("q"),
            ord("Q"),
            27
        ):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
