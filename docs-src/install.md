# Install

## From apt

CUDA-enabled packages for Humble, Jazzy, Kilted, Lyrical and Rolling (amd64). They need only
the NVIDIA driver, not the CUDA toolkit, and run on the CPU on machines without a usable GPU.
Lyrical needs driver 580 or newer, the rest 525 or newer.

```bash
sudo curl -fsSL https://polka-ros.github.io/polka.gpg -o /usr/share/keyrings/polka.gpg
echo "deb [signed-by=/usr/share/keyrings/polka.gpg] https://polka-ros.github.io $(. /etc/os-release && echo $VERSION_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/polka.list
sudo apt update && sudo apt install ros-$ROS_DISTRO-polka
```

## From source

One branch per ROS 2 distro, same code on each:

| Distro | Ubuntu | Branch |
|--------|--------|--------|
| Humble | 22.04 | [`humble`](https://github.com/Pana1v/polka/tree/humble) |
| Iron | 22.04 | [`iron`](https://github.com/Pana1v/polka/tree/iron) |
| Jazzy | 24.04 | [`jazzy`](https://github.com/Pana1v/polka/tree/jazzy) |
| Kilted | 24.04 | [`kilted`](https://github.com/Pana1v/polka/tree/kilted) |
| Lyrical | 26.04 | [`lyrical`](https://github.com/Pana1v/polka/tree/lyrical) |
| Rolling | 26.04 | [`rolling`](https://github.com/Pana1v/polka/tree/rolling) |

```bash
git clone -b humble https://github.com/Pana1v/polka.git ~/ros2_ws/src/polka
cd ~/ros2_ws && colcon build --packages-select polka
# the CUDA engine is built when nvcc is found; -DWITH_CUDA=OFF forces CPU-only
```

## Quick start

```bash
cp config/example_params.yaml config/my_robot.yaml      # edit topics + output_frame_id
ros2 launch polka polka.launch.py config_file:=config/my_robot.yaml
```

Point `output_frame_id` at your base frame, list your sensors under `source_names`, and check that TF resolves every sensor `frame_id` to `output_frame_id`. Playing a bag? Pass `use_sim_time:=true` and play with `--clock` (see [Configuration](configuration.md#rosbag--simulation-playback)).
