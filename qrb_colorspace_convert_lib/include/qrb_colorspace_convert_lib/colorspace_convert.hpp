// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_COLORSPACE_CONVERT_LIB__COLORSPACE_CONVERT_HPP_
#define QRB_COLORSPACE_CONVERT_LIB__COLORSPACE_CONVERT_HPP_

#include "qrb_colorspace_convert_lib/opengles_common.hpp"

namespace qrb::colorspace_convert_lib
{
class OpenGLESAccelerator
{
public:
  OpenGLESAccelerator();
  ~OpenGLESAccelerator();

  /// Convert NV12 to RGB8
  /// @param in_fd input DMA_BUF file descriptor
  /// @param out_fd output DMA_BUF file descriptor
  /// @param width image width, need align with GPU supported size
  /// @param height image height, need align with GPU supported size
  bool nv12_to_rgb8(int in_fd, int out_fd, int width, int height);

  /// Convert RGB8 to NV12
  /// @param in_fd input DMA_BUF file descriptor
  /// @param out_fd output DMA_BUF file descriptor
  /// @param width image width, need align with GPU supported size
  /// @param height image height, need align with GPU supported size
  bool rgb8_to_nv12(int in_fd, int out_fd, int width, int height);

private:
  bool egl_init();
  bool egl_deinit();

  EGLDisplay display_;
  EGLContext context_;
  GLuint framebuffer_;
  bool initialized_;
  GLuint textures_[2];
  GLProgram gl_program_;

  EGLImageKHR src_img;
  EGLImageKHR out_img;
};

}  // namespace qrb::colorspace_convert_lib

#endif  // QRB_COLORSPACE_CONVERT_LIB__COLORSPACE_CONVERT_HPP_
