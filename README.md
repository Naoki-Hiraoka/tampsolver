# Install
Follow the instruction of README.md of rtmros_hrp2.

You need

- https://github.com/jsk-ros-pkg/jsk_model_tools/pull/231
- https://github.com/start-jsk/rtmros_hrp2/pull/555
- https://github.com/start-jsk/rtmros_common/pull/1099
- https://github.com/fkanehiro/hrpsys-base/pull/1294
- https://github.com/fkanehiro/hrpsys-base/pull/1295
- https://github.com/jsk-ros-pkg/jsk_roseus/pull/652
- https://github.com/choreonoid/choreonoid/pull/12

Then,

```
wstool merge -t src src/tampsolver/.rosinstall
wstool update -t src
```

```
rosdep install -r --from-paths src --ignore-src -y
```

```
./src/choreonoid/misc/script/install-requisites-ubuntu-18.04.sh
patch -p1 -d src/choreonoid < src/tampsolver/symbol_geometry_planner/patch/choreonoid.patch
```

Follow the instruction of eusurdfwrl.

Then,

```
catkin build multicontact_controller
```