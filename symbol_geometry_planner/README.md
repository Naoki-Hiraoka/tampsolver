# Install

```
git clone https://github.com/s-nakaoka/choreonoid --branch release-1.7
./choreonoid/misc/script/install-requisites-ubuntu-16.04.sh
patch -p1 -d choreonoid < [path to symbol_geometry_planner]/patch/choreonoid.patch
```

```
rosdep install -r --from-paths tampsolver --ignore-src -y
```