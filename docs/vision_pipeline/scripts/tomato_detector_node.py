#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shlex
import subprocess
import sys

import rospkg


def main():
    conda_sh = os.environ.get("CONDA_SH", "/home/zmy/miniconda3/etc/profile.d/conda.sh")
    conda_env = os.environ.get("CONDA_ENV", "yolov9")

    if not os.path.isfile(conda_sh):
        print("Conda profile script not found: {}".format(conda_sh), file=sys.stderr)
        return 1

    script_path = os.path.join(
        rospkg.RosPack().get_path("vision_double_arm_ros1"),
        "scripts",
        "tomato_detector_core.py",
    )
    forwarded_args = " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    command = (
        "source {} && "
        "conda activate {} && "
        "exec python3 {} {}"
    ).format(
        shlex.quote(conda_sh),
        shlex.quote(conda_env),
        shlex.quote(script_path),
        forwarded_args,
    )
    return subprocess.call(["bash", "-lc", command])


if __name__ == "__main__":
    raise SystemExit(main())
