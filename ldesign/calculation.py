import pathlib

import numpy as np
import scipy.constants


def calc_wave_length(freq, e=5.749):
    return scipy.constants.c / np.sqrt(e) / freq


def calc_freq(wave_len, e=5.749):
    return scipy.constants.c / np.sqrt(e) / wave_len


if __name__ == '__main__':
    print(calc_wave_length(5.561085e9)/4*1e6 - 5200)
    print(calc_freq(20.5e-3*2))
    print(np.cos(8000/(calc_wave_length(4.5e9)*1e6/2)*np.pi/2))
    print(8000 / (calc_wave_length(4.5e9) * 1e6 / 2) * 90)
