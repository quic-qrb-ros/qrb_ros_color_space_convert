# QRB Color Space Conversion ROS Node

Provide ROS node to convert nv12 to rgb8 each other. 

## Overview

Qualcomm's smart devices, such as the RB3 Gen2, use NV12 as the default image color space conversion format. Generally, the more common color space format is RGB888. To embrace open source and facilitate developers in converting between these two formats, we have developed the color space conversion ROS node. The feature as follows:

- Provide ROS node include
  - API to convert nv12 to rgb8
  - API to convert rgb8 to nv12

- Support dmabuf fd as input / output

- Input / output image receive/send with QRB ROS transport
- Hardware accelerates with GPU by OpenGL ES

## Build

Currently, we only support two color space format: NV12 and RGB888.

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`
     ```bash
     git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_color_space_convert.git
     ```
4. Build this project
     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

     colcon build --merge-install --cmake-args \
       -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF
     ```
5. Push to the device & Install
     ```bash
     cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
     tar czvf qrb_ros_colorspace_convert.tar.gz lib share
     scp qrb_ros_colorspace_convert.tar.gz root@[ip-addr]:/opt/
     ssh root@[ip-addr]
     (ssh) tar -zxf /opt/qrb_ros_colorspace_convert.tar.gz -C /opt/qcom/qirp-sdk/usr/
     ```
## Run

- Source this file to set up the environment on your device:

```bash
ssh root@[ip-addr]
(ssh) export XDG_RUNTIME_DIR=/dev/socket/weston/
(ssh) export WAYLAND_DISPLAY=wayland-1
```

Run the ROS2 package.

```
(ssh) ros2 launch qrb_ros_colorspace_convert colorspace_convert.launch.py
```

- You can modify the colorspace_convert.launch.py to convert the color space type.

```python
def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        name='component_colorconvert_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='qrb_ros_colorspace_convert',
                plugin='qrb_ros::colorspace_convert::ColorspaceConvertNode',
                parameters=[{
                    'conversion_type': "nv12_to_rgb8", # or rgb8_to_nv12
                    'latency_fps_test': True, # False to disable test
                }],
                extra_arguments=[{'use_intra_process_comms': True, 'log_level': 'INFO'}],
            ),
        ]
    )])
```

## Acceleration

The convert process accelerate by OpenGL ES on GPU.

## Packages

Will update in the future.

## Resources

- [ROS2 Type Adaption](https://ros.org/reps/rep-2007.html)

## Contributions

Thanks for your interest in contributing to qrb_colorspace_convert_lib! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_colorspace_convert_lib is licensed under the BSD-3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.
