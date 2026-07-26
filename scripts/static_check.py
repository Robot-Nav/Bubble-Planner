#!/usr/bin/env python3
"""Dependency-free structural checks for the source package."""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
required = [
    "package.xml", "CMakeLists.txt",
    "src/bubble_planner_node.cpp", "src/corridor_generator.cpp",
    "src/minco_optimizer.cpp", "config/bubble_paper.yaml",
    "launch/bubble_planner.launch", "README.md",
]
missing = [name for name in required if not (root / name).is_file()]
if missing:
    print("Missing files:", *missing, sep="\n  - ")
    sys.exit(1)

for path in list((root / "src").glob("*.cpp")) + list((root / "include").rglob("*.hpp")):
    text = path.read_text(encoding="utf-8")
    if text.count("{") != text.count("}"):
        print(f"Unbalanced braces: {path}")
        sys.exit(2)

print("Structural checks passed.")
