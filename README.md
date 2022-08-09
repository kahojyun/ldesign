# LDesign

Layout design tool for superconducting circuits based on
[gdstk](https://github.com/heitzmann/gdstk).


## Installation

Currently, the `ldesign` project only supports `python >= 3.10`.  Because
`gdstk` and `gdspy` is difficult to install on Windows via `pip`, Windows users are
suggested to use `conda`.
```
conda create -n ldesign -c conda-forge --strict-channel-priority python=3.10
conda activate ldesign
conda install gdstk gdspy
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


## Additional Folders

* `typings` stores type stubs for `gdstk`



## To-do
* Shared element factory.
* Element reference.
* Element builder.
* Simplified `PathBuilder`.