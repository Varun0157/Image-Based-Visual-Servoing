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

#### Demo 
<table>
    <tr>
        <td style="width:50%">
            <img src="demo/target.png" alt="Target Image" style="width: 100%;">
        </td>
        <td style="width:50%">
            <video style="width:100%" controls>
                <source src="demo/output.mp4" type="video/mp4">
                Your browser does not support the video tag.
            </video>
        </td>
    </tr>
</table>

### TODO
- [ ] experiment with more complex dof. Currently simply taking the first three points' (x, y) coords as our 6 dof. 
- [ ] find out why the obstacles fall slowly in the p.DIRECT sim even though it does not seem to happen in p.GUI. 
