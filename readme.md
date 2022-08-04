# LDesign

Layout design tool for superconducting circuits based on
[gdstk](https://github.com/heitzmann/gdstk).

## Installation

Currently, the `ldesign` project only supports `python >= 3.10`.  Because
`gdstk` and `gdspy` is difficult to install on Windows via `pip`, users are
encouraged to use `conda`.
```
conda create -n gdstk -c conda-forge --strict-channel-priority python=3.10
conda activate gdstk
conda install gdstk
```
Refer to [gdstk](https://github.com/heitzmann/gdstk) and
[gdspy](https://github.com/heitzmann/gdspy) for details.


After installing `gdstk` and `gdspy`, install `ldesign` from pip
```
pip install ldesign
```
or install in development mode
```
git clone https://github.com/kahojyun/ldesign.git
cd ldesign
pip install -e .
```



## To-do
* Shared element factory.
* Element reference.
* Element builder.
* Simplified `PathBuilder`.