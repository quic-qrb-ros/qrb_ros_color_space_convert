// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_colorspace_convert_lib/colorspace_convert.hpp"
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <drm/drm_fourcc.h>

#define ION_SECURE_HEAP_ALIGNMENT (0x100000)
#define ALIGN(x, y) (((x) + (y)-1) & (~((y)-1)))

static int alloc_dma_buf(int size)
{
  int heap_fd = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
  if (heap_fd < 0) {
    std::cerr << "open dma heap failed" << std::endl;
    return -1;
  }

  struct dma_heap_allocation_data heap_data = {};
  heap_data.len = size;
  heap_data.fd_flags = O_RDWR | O_CLOEXEC;

  if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &heap_data) != 0) {
    std::cerr << "dma heap alloc failed, len: " << heap_data.len << std::endl;
    close(heap_fd);
    return -1;
  }
  close(heap_fd); // Close heap_fd after allocation
  return heap_data.fd;
}

static int create_aligned_image(int width, int height, int format, int size)
{
  int fd = alloc_dma_buf(size);
  if (fd <= 0) {
    return -1;
  }
  char *dst = (char *)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (dst == MAP_FAILED) {
    std::cerr << "storage data: mmap failed" << std::endl;
    close(fd);
    return -1;
  }

  for (int i = 0; i < size; ++i) {
    dst[i] = static_cast<char>(i % 256); // 简单的填充数据
  }

  munmap(dst, size);
  return fd;
}

static void dump_data_to_file(int fd, int size, const std::string &path)
{
  char *dst = (char *)mmap(NULL, size, PROT_READ, MAP_SHARED, fd, 0);
  if (dst == MAP_FAILED) {
    std::cerr << "dump_data_to_file: mmap failed" << std::endl;
    close(fd);
    return;
  }

  std::ofstream out(path, std::ios::out | std::ios::binary);
  if (out.is_open()) {
    out.write(dst, size);
  } else {
    std::cerr << "open file: " << path << " failed" << std::endl;
  }
  out.close();

  munmap(dst, size);
  close(fd);
}

int test_nv12_to_rgb8()
{
  int width = 1920;
  int height = 1080;

  int align_height = ALIGN(height, 1);
  int align_width = ALIGN(width, 64);

  std::cout << "align_height:" << align_height << std::endl;
  std::cout << " align_width:" << align_width << std::endl;

  int input_fd = create_aligned_image(align_width, align_height, DRM_FORMAT_NV12, align_width * align_height * 4);
  int output_fd = alloc_dma_buf(align_width * align_height * 4);

  std::cout << "infd: " << input_fd << ", out fd: " << output_fd << std::endl;

  if (input_fd < 0 || output_fd < 0) {
    std::cout << "BufferAllocator::Alloc failed" << std::endl;
    return 1;
  }

  qrb::colorspace_convert_lib::OpenGLESAccelerator accelerator;

  bool success = accelerator.nv12_to_rgb8(input_fd, output_fd, align_width, align_height);
  if (!success) {
    std::cerr << "nv12 to rgb8 failed" << std::endl;
  } else {
    std::cout << "nv12 to rgb8 success" << std::endl;
    dump_data_to_file(output_fd, align_width * align_height * 4, "/data/dst.rgb8");
  }

  close(input_fd);
  close(output_fd);

  return 0;
}

int test_rgb8_to_nv12()
{
  int width = 1920;
  int height = 1080;

  int align_height = ALIGN(height, 1);
  int align_width = ALIGN(width, 256);

  std::cout << "align_height:" << align_height << std::endl;
  std::cout << " align_width:" << align_width << std::endl;

  int input_fd = create_aligned_image(align_width, align_height, DRM_FORMAT_BGR888, align_width * align_height * 4);
  int output_fd = alloc_dma_buf(align_width * align_height * 8);

  if (input_fd < 0 || output_fd < 0) {
    std::cout << "BufferAllocator::Alloc failed" << std::endl;
    return 1;
  }

  qrb::colorspace_convert_lib::OpenGLESAccelerator accelerator;
  bool success = accelerator.rgb8_to_nv12(input_fd, output_fd, align_width, align_height);
  if (!success) {
    std::cerr << "rgb8 to nv12 failed" << std::endl;
  } else {
    std::cout << "rgb8 to nv12 success" << std::endl;
    dump_data_to_file(output_fd, align_width * align_height * 4, "/data/dst.yuv");
  }

  close(input_fd);
  close(output_fd);

  return 0;
}

int main()
{
  test_nv12_to_rgb8();
  test_rgb8_to_nv12();
  return 0;
}