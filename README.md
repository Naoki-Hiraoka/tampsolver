# Install
Follow the instruction of README.md of rtmros_hrp2.

You need

- https://github.com/jsk-ros-pkg/jsk_model_tools/pull/231
- https://github.com/start-jsk/rtmros_hrp2/pull/555
- https://github.com/choreonoid/choreonoid/pull/7
- https://github.com/start-jsk/rtmros_common/pull/1099
- https://github.com/fkanehiro/hrpsys-base/pull/1294

Then,

```
wstool merge -t src src/tampsolver/.rosinstall
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