# Image Based Visual Servoing

A simple IBVS simulation in `pybullet` built as a learning experiment as a part of my undergraduate research.

The initial code was built off of @roadrollerdafjorst's [IBVS sim](https://github.com/roadrollerdafjorst/visual-servoing/tree/main/IBVS) but has deviated since.

## Instructions

- `docs/env.yml` contains the conda environment config file.
  Create an environment (here, called `ibvs`) and load it in as usual:

```bash
cd docs
conda env create -f env.yml
conda activate ibvs
```

- run the pybullet sim in `DIRECT` mode, with images saved to `src/img/`.

```bash
cd src
python main.py
```

- if required, convert the images into an MP4 file for later analysis

```bash
bash create_vid.sh
```

## Demo

| <img src="demo/target.png" width="700" /> | <img src="demo/output.gif" width="600" /> |
| :---------------------------------------: | :---------------------------------------: |
|               Target Image                |                Output GIF                 |

## TODO

- [ ] refactor: get rid of globals and replace with a config, remove hard-coded paths
- [ ] add a scheduler for `LAMBDA`
- [ ] use a better target and scene
- [ ] experiment with more complex dof.
  - Currently simply taking the first three points' (x, y) coords as our 6 dof.
- [ ] find out why the obstacles fall slowly in the p.DIRECT sim even though it does not seem to happen in p.GUI.
- [ ] port to `airobot` to get rid of view matrix complexity
