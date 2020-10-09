# Install
Follow the instruction of README.md of rtmros_hrp2.

Then,

```
wstool merge -t src src/tampsolver/.rosintall
wstool update -t src
```

```
rosdep install -r --from-paths src --ignore-src -y
```

```
./src/choreonoid/misc/script/install-requisites-ubuntu-16.04.sh
patch -p1 -d src/choreonoid < src/tampsolver/symbol_geometry_planner/patch/choreonoid.patch
```

Follow the instruction of eusurdfwrl.

Then,

```
catkin build symbol_geometry_planner
```