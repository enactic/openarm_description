# OpenArm Reachability Benchmark

This tool benchmarks the reachability workspace of the OpenArm robot for different end effector orientations.
This tool is designed to visualize the current reachability given the URDF file and leverage it for future improvements and customization of the OpenArm's mechanical structure.

## Usage
Run a single benchmark by specifying height, direction
```bash
uv run bench.py --z 0.5 --direction forward1
```
z is the height of the target coordinates, and the reachability map is computed for the plane at height z.

Execute batch benchmarks across multiple heights and directions:
```bash
./batch_bench.sh
```

Visualize results with animation support:
```bash
uv run visualize.py --direction forward1 --htable 0.29 --animate
```
