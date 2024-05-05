### Instructions
- `src/env.yml` contains the conda environment config file. Create an environment (here, called `ibvs`) and load it in as usual:
```bash 
cd src 
conda env create -f env.yml
conda activate ibvs  
```
- run the pybullet sim in `DIRECT` mode, with images saved to `src/img/`. 
```bash
cd src 
python index.py
```
- if required, convert the images into an MP4 file for later analysis, storing the video in `src/vid/`
```
bash create_vid.sh
```

various factors can be altered in the `src/index.py` file, such as 
