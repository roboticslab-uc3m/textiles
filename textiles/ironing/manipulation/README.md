# README.md

1. From teoBase, launch:
```bash
launchManipulation --externalEncoderWait 1
yarpdev --device Jr3 --period 20 --name /jr3 --ports "(ch0:o ch1:o ch2:o ch3:o)" --channels 24 --ch0:o 0 5 0 5 --ch1:o 6 11 0 5 --ch2:o 12 17 0 5 --ch3:o 18 23 0 5
launchLocomotion --externalEncoderWait 1
```

2. From teoTools, test with:
```bash
yarpmotorgui --from config/yarpmotorgui.ini
yarpscope --remote /jr3/ch2:o --index "(0 1 2 3 4 5)" --color "(Red Green Blue LightRed LightGreen LightBlue)" --min -1000 --max 1000 --x 0 --y 0 --dx 640 --dy 512
```

- From app, launch:
```bash
yarpdev --from /usr/local/share/teo/contexts/kinematics/rightArmKinematics-pan45-tilt30.ini --name /teo/rightArm/CartesianControl --device BasicCartesianControl --angleRepr axisAngle --robot remote_controlboard --local /BasicCartesianControl/teo/rightArm --remote /teo/rightArm
```

- Run `ironingMover --robot /teo --avoidTrunk 1 --targetForce -5.0 --strategy position` # Use `--help` parameter for more options and other strategies.

- From app, connect all.
