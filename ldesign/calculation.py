import math

import numpy as np
import scipy.constants

_phi0: float = scipy.constants.value("mag. flux quantum")
_c: float = scipy.constants.c
_h: float = scipy.constants.h


def calc_wave_length(freq: float, e: float = 5.749):
    return _c / math.sqrt(e) / freq


def calc_freq(wave_len: float, e: float = 5.749):
    return _c / math.sqrt(e) / wave_len


def e_from_freq(freq: float):
    return freq * _h


def ic_from_ej(ej: float, r_ratio: float = 1.19):
    return math.tau * ej / _phi0 * r_ratio


def r_from_ic(ic: float, v_gap: float = 0.28e-3):
    return v_gap / ic


def area_from_ic(ic: float, jc: float = 520e3):
    return ic / jc


def area_from_ej(ej: float, r_ratio: float = 1.19, jc: float = 520e3):
    return area_from_ic(ic_from_ej(ej, r_ratio), jc)


if __name__ == "__main__":
    print(calc_wave_length(6e9))
    print(calc_wave_length(5.561085e9) / 4 * 1e6 - 5200)
    print(calc_freq(20.5e-3 * 2))
    print(np.cos(8000 / (calc_wave_length(4.5e9) * 1e6 / 2) * np.pi / 2))
    print(8000 / (calc_wave_length(4.5e9) * 1e6 / 2) * 90)
