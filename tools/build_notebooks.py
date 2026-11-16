#!/usr/bin/env python3
"""维护者工具: 把 tools/nbsrc/*.py (percent 格式) 生成为 notebooks/*.ipynb。

为什么这样做: .ipynb 是 JSON，不好评审也不好合并；用纯文本写、生成后提交。
学生和老师只需要 notebooks/ 里的 .ipynb，用不到这个脚本。

    python3 tools/build_notebooks.py            # 全部重新生成
    python3 tools/build_notebooks.py 06         # 只生成编号以 06 开头的

格式:
    # %% [markdown]
    # 这里是 Markdown (每行去掉开头的 "# ")
    # %%
    代码单元
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "tools" / "nbsrc"
OUT = ROOT / "notebooks"


def parse(text: str):
    cells, kind, buf = [], None, []

    def flush():
        if kind is None:
            return
        lines = buf[:]
        while lines and not lines[-1].strip():
            lines.pop()
        while lines and not lines[0].strip():
            lines.pop(0)
        if kind == "markdown":
            lines = [l[2:] if l.startswith("# ") else ("" if l.strip() == "#" else l) for l in lines]
        if lines:
            cells.append((kind, "\n".join(lines)))

    for line in text.splitlines():
        if line.startswith("# %%"):
            flush()
            kind = "markdown" if "[markdown]" in line else "code"
            buf = []
        else:
            buf.append(line)
    flush()
    return cells


def to_notebook(cells, stem: str):
    nb_cells = []
    for i, (kind, src) in enumerate(cells):
        cid = hashlib.md5(f"{stem}-{i}".encode()).hexdigest()[:12]
        cell = {"cell_type": kind, "id": cid, "metadata": {}, "source": src.splitlines(keepends=True)}
        if kind == "code":
            cell.update(execution_count=None, outputs=[])
        nb_cells.append(cell)
    return {
        "cells": nb_cells,
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python"},
        },
        "nbformat": 4, "nbformat_minor": 5,
    }


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else ""
    OUT.mkdir(exist_ok=True)
    n = 0
    for src in sorted(SRC.glob(f"{prefix}*.py")):
        nb = to_notebook(parse(src.read_text()), src.stem)
        (OUT / f"{src.stem}.ipynb").write_text(json.dumps(nb, ensure_ascii=False, indent=1) + "\n")
        n += 1
        print(f"  {src.stem}.ipynb  ({len(nb['cells'])} cells)")
    print(f"生成 {n} 个 Notebook -> {OUT.relative_to(ROOT)}/")


if __name__ == "__main__":
    main()
