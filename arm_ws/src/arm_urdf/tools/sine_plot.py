#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sine_plot.py —— 把 sine_test.py 的 CSV 画成图

用法：python3 sine_plot.py sine_test.csv out.png [电机ID]
"""
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "sine_test.csv"
    out = sys.argv[2] if len(sys.argv) > 2 else "sine_test.png"
    only = int(sys.argv[3]) if len(sys.argv) > 3 else None

    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        print("空文件")
        return 1

    def col(name):
        out_v = []
        for r in rows:
            v = r.get(name, "")
            out_v.append(float(v) if v not in ("", "None") else float("nan"))
        return out_v

    t = col("t")
    cmd = [math.degrees(v) for v in col("cmd")]
    ids = sorted({int(k.split("_")[1]) for k in rows[0] if k.startswith("pos_")})
    if only is not None:
        ids = [i for i in ids if i == only]

    n = max(1, len(ids))
    fig, axes = plt.subplots(n, 2, figsize=(13, 2.6 * n), squeeze=False)

    for idx, mid in enumerate(ids):
        pos = [math.degrees(v) for v in col("pos_%d" % mid)]
        iq = col("iq_%d" % mid)

        ax = axes[idx][0]
        ax.plot(t, cmd, "k--", lw=1, label="cmd")
        ax.plot(t, pos, "C0", lw=1.2, label="actual")
        ax.set_ylabel("motor %d (deg)" % mid)
        ax.grid(alpha=0.3)
        if idx == 0:
            ax.legend(fontsize=8)
            ax.set_title("position tracking", fontsize=10)

        ax2 = axes[idx][1]
        err = [p - c for p, c in zip(pos, cmd)]
        ax2.plot(t, err, "C3", lw=1)
        ax2.set_ylabel("err (deg)")
        ax2.grid(alpha=0.3)
        if idx == 0:
            ax2.set_title("tracking error", fontsize=10)
        ax3 = ax2.twinx()
        ax3.plot(t, iq, "C2", lw=0.8, alpha=0.6)
        ax3.set_ylabel("iq (A)", color="C2", fontsize=8)

    for ax in axes[-1]:
        ax.set_xlabel("time (s)")

    vbus = col("vbus")
    if not all(math.isnan(v) for v in vbus):
        fig.suptitle("sine test  |  vbus min %.1f V max %.1f V" % (min(v for v in vbus if not math.isnan(v)),
                                                                   max(v for v in vbus if not math.isnan(v))),
                     fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=140)
    print("saved", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
