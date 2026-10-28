# Development

## How to test

Run `test/run.sh` with a build directory in a ROS 2 environment:

```bash
source /opt/ros/jazzy/setup.bash # Use your ROS 2 distribution
test/run.sh /tmp/openarm_description.build
```

You can also use a ROS 2 Docker image like CI does. Run this at the
top directory of this repository:

```bash
docker container run \
  --rm \
  --volume ${PWD}:/source \
  osrf/ros:jazzy-desktop-full \
  /source/test/run.sh /tmp/build
```

## How to release

```bash
git clone git@github.com:enactic/openarm_description.git
cd openarm_description
dev/release.sh ${VERSION} # e.g. dev/release.sh 1.0.0
```
