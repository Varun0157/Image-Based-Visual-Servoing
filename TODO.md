# Formatting
- involve depth calculation in interaction mat
  - observation: divergence decreases massively. Maybe now we can rely on the MAX_ITERATIONS as tell all. Try using sbatch. 
  - Now, you can try more complex features as it will keep trying to converge
- more complex features (eg: account for the fourth point by making your 6 dof $(x_1, y_1), (x_2, y_2) and (\frac{x_3 + x_4}{2}, \frac{y_3 + y_4}{2})$). 
- figure out why the obstacles are falling in p.DIRECT