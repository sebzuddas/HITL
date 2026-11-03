#!/usr/bin/env python3
"""Convert the MATLAB-generated controller files in data/ into web/data/controller.json.

Run from anywhere:  python3 scripts/export_web_data.py
Only the standard library is used so it works in CI without installs.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
OUT = ROOT / "web" / "data" / "controller.json"


def read_matrix(name):
    rows = []
    for line in (DATA / name).read_text().strip().splitlines():
        rows.append([float(v) for v in line.replace("\t", ",").split(",") if v.strip()])
    return rows


def read_vector(name):
    return [v for row in read_matrix(name) for v in row]


PARAM_NAMES = ["T", "U_e", "l1", "l2", "l3", "Jp", "Jt", "Je", "m1", "m2", "g",
               "k_amp", "amp_sat_u", "amp_sat_l", "DAC_lim_u", "DAC_lim_l",
               "e_lim_l", "e_lim_u", "p_lim_l", "p_lim_u"]  # order written by heli.m

params = dict(zip(PARAM_NAMES, read_vector("ModelParameters.txt")))
fan = read_vector("FanChar.txt")  # heli.m writes [Vfan, Thrust] flattened: first half V, second half F
half = len(fan) // 2

out = {
    "params": params,
    "fan": {"volts": fan[:half], "thrust": fan[half:]},
    "kf": {k: read_matrix(f"{k}Matrix.txt") for k in ("a", "b", "c", "d")},
    "Kd": read_matrix("KdMatrix.txt"),
    "Kdi": read_matrix("KdiMatrix.txt"),
    "Cr": read_matrix("crMatrix.txt"),
    "kfInitial": read_vector("KF_Initial.txt"),
    "kdiInitial": read_vector("Kdi_Initial.txt"),
}
OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(json.dumps(out, indent=1))
print(f"wrote {OUT.relative_to(ROOT)}")
