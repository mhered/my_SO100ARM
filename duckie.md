# Adding a duckie

TEMPORARY: this is really for the pycamp project but I tried to prototype it in `bar_ws`

Two things ChatGPT suggested and didn't work:

```bash
gz model --spawn-file=/home/mhered/bar_ws/src/duckie_model/model.sdf --model-name duckie
```

```bash
mkdir -p ~/.gazebo/models
cp -r /home/mhered/bar_ws/src/duckie_model/ ~/.gazebo/models/
```

Ended up manually writing a model folder by copy & pasting those in gamecity package as templates

to test how to add a duckie I modified simple_gamecity to add the duckie. 

.obj does show without beak and floats strangely!!

.dae crashes with segmentation fault

Why? maybe because this model it is not a solid? maybe wrong inertias?



closed the solid

fixed the floating it was wrong inertias

now head does not show!!

why krytn does not move??