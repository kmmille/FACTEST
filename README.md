# Everything is developed in Python3.9

# Required libraries
- numpy
- polytope
- z3-solver
- matplotlib

# Optional libraries
- yices

# Testing base factest file
```
python3.9 factest/synthesis/factest_base.py
```

# Testing examples from CAV20
```
python3.9 demo/demo_cav20.py [model] [env] [--solver] [--segs] [--parts] [--print] [--plot]
```

# Environments which still need to be converted:
- demo.py
- easy.py
- hard.py
- Ltunnel.py
- maze_3d.py
- office.py
- partition2.py (rename to partition_2d.py)
- zigzag.py
- zigzag2.py
- zigzag3.py
- ztunnel.py