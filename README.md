# QRB Color Space Conversion ROS Node

## Overview

Qualcomm's smart devices, such as the RB3 Gen2, use NV12 as the default image color space conversion format. Generally, the more common color space format is RGB888. To embrace open source and facilitate developers in converting between these two formats, we have developed the color space conversion ROS node. The feature as follows:

- Provide ROS node include
  - API to convert nv12 to rgb8
  - API to convert rgb8 to nv12
- Support dmabuf fd as input / output
- Input / output image receive/send with QRB ROS transport
- Hardware accelerates with GPU by OpenGL ES

## Documentation



Please refer to the [QRB ROS Documentation](https://quic-qrb-ros.github.io/) for more documents.

- [Getting Started](https://quic-qrb-ros.github.io/getting_started/index.html)
- [QRB ROS Packages](https://quic-qrb-ros.github.io/packages/index.html)
- [Release Notes](https://quic-qrb-ros.github.io/release_notes/index.html)



## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/blob/QRBROS/CONTRIBUTING.md) and [code of conduct](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/blob/QRBROS/CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/issues)

## Documentation

Please visit [QRB ROS Documentation](https://quic-qrb-ros.github.io/) for more details.

## Authors

- **Vito Wang** - *Initial work* - [violet227 (Vito Wang)](https://github.com/violet227)

See also the list of [Contributors to quic-qrb-ros/qrb_ros_color_space_convert](https://github.com/quic-qrb-ros/qrb_ros_color_space_convert/graphs/contributors) who participated in this project.

## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](https://github.qualcomm.com/jiaxshi/QRB-ROS-repository-template/blob/QRBROS/LICENSE) for the full license text.