#!/usr/bin/env python3
"""Apply Foxy-compatibility patches to the C++ sources of this workspace.

Each patch is idempotent: running the script twice is a no-op. The script
reports every edit it makes so the result can be reviewed before building.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

CXX_SUFFIXES = {".cpp", ".cc", ".h", ".hpp", ".hxx"}


def cxx_files():
    for p in SRC.rglob("*"):
        if p.suffix in CXX_SUFFIXES and p.is_file():
            yield p


HPP_INCLUDE = re.compile(r'^[ \t]*#include[ \t]*([<"])([^>"]+\.hpp)([>"])[ \t]*$')

# Headers that Humble ships as <name.hpp> but Foxy only ships as <name.h>.
# laser_geometry is the opposite way round (Foxy has only the .hpp), so it is
# deliberately absent from this list.
HPP_GUARDED = (
    "tf2_geometry_msgs/tf2_geometry_msgs",
    "tf2/utils",
    "tf2_eigen/tf2_eigen",
    "cv_bridge/cv_bridge",
    "image_geometry/pinhole_camera_model",
)


def patch_hpp_includes(text):
    """Make <name.hpp> includes work on both Humble and Foxy.

    For the headers in HPP_GUARDED, Humble ships <body.hpp> while Foxy only
    ships <body.h>. Includes that already sit inside a __has_include guard are
    left alone so the pass stays idempotent.
    """
    lines = text.splitlines(keepends=True)
    out = []
    edits = 0
    for i, line in enumerate(lines):
        m = HPP_INCLUDE.match(line.rstrip("\n"))
        body = m.group(2)[:-4] if m else None  # strip ".hpp"
        if not m or body not in HPP_GUARDED:
            out.append(line)
            continue
        # Already wrapped by a previous run?
        prev = next((l for l in reversed(out) if l.strip()), "")
        if "__has_include" in prev:
            out.append(line)
            continue
        open_d, close_d = m.group(1), m.group(3)
        out.append(
            f"#if __has_include({open_d}{body}.hpp{close_d})\n"
            f"#include {open_d}{body}.hpp{close_d}\n"
            f"#else\n"
            f"#include {open_d}{body}.h{close_d}\n"
            f"#endif\n"
        )
        edits += 1
    return "".join(out), edits


# Any `<node>->now()` where <node> is an rclcpp::Node. The lookaheads skip
# get_clock()->now() (keeps the pass idempotent) and rclcpp::Clock receivers
# such as `clock_->now()`, which already exist on Foxy.
NODE_NOW = re.compile(
    r'(?<![\w.])(?!get_clock\b)(?!clock_\b)([A-Za-z_]\w*)\s*->\s*now\s*\(\s*\)'
)


def patch_node_now(text):
    """rclcpp::Node::now() was added in Galactic; Foxy needs get_clock()->now()."""
    text, n1 = NODE_NOW.subn(lambda m: f"{m.group(1)}->get_clock()->now()", text)
    text, n2 = re.subn(r'\bthis\.now\s*\(\s*\)', 'this.get_clock()->now()', text)
    return text, n1 + n2


# Files whose classes derive from rclcpp::Node and call now() unqualified.
BARE_NOW_FILES = (
    "scan_planner_ros2/src/planner/plan_manage/src/closed_loop_controller.cpp",
    "scan_planner_ros2/src/planner/plan_manage/src/go2_kinematic_sim.cpp",
    "jie_3d_nav/octo_planner/src/test_map_to_odom_tf_node.cpp",
)

# now() with no receiver; the lookbehind keeps already-patched calls idempotent.
BARE_NOW = re.compile(r"(?<![>\w.])\bnow\s*\(\s*\)")


def patch_bare_now(text):
    out_lines = []
    edits = 0
    for line in text.splitlines(keepends=True):
        stripped = line.lstrip()
        if stripped.startswith("//") or stripped.startswith("*"):
            out_lines.append(line)
            continue
        new, n = BARE_NOW.subn("get_clock()->now()", line)
        edits += n
        out_lines.append(new)
    return "".join(out_lines), edits


def main():
    total = 0
    report = []
    for path in sorted(cxx_files()):
        rel = str(path.relative_to(ROOT))
        original = path.read_text(errors="surrogateescape")

        text, e_hpp = patch_hpp_includes(original)
        text, e_now = patch_node_now(text)
        e_bare = 0
        if rel.endswith(BARE_NOW_FILES):
            text, e_bare = patch_bare_now(text)

        if text != original:
            path.write_text(text, errors="surrogateescape")
            report.append(f"{rel}: hpp-include={e_hpp} node->now={e_now} bare-now={e_bare}")
            total += e_hpp + e_now + e_bare

    print(f"total edits: {total}")
    for line in report:
        print("  " + line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
