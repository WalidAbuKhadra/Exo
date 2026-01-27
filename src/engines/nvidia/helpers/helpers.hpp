#pragma once

#include <nvCVImage.h>
#include <opencv2/core/mat.hpp>

namespace estimation::nvidia::helpers {

inline void NVWrapperForCVMat(const cv::Mat *cvIm, NvCVImage *nvcvIm) {
  static const NvCVImage_PixelFormat nvFormat[] = {NVCV_FORMAT_UNKNOWN, NVCV_Y, NVCV_YA, NVCV_BGR, NVCV_BGRA};
  static const NvCVImage_ComponentType nvType[] = {NVCV_U8, NVCV_TYPE_UNKNOWN, NVCV_U16, NVCV_S16, NVCV_S32, NVCV_F32, NVCV_F64, NVCV_TYPE_UNKNOWN};
  nvcvIm->pixels = cvIm->data;
  nvcvIm->width = cvIm->cols;
  nvcvIm->height = cvIm->rows;
  nvcvIm->pitch = (int)cvIm->step[0];
  nvcvIm->pixelFormat = nvFormat[cvIm->channels() <= 4 ? cvIm->channels() : 0];
  nvcvIm->componentType = nvType[cvIm->depth() & 7];
  nvcvIm->bufferBytes = 0;
  nvcvIm->deletePtr = nullptr;
  nvcvIm->deleteProc = nullptr;
  nvcvIm->pixelBytes = (unsigned char)cvIm->step[1];
  nvcvIm->componentBytes = (unsigned char)cvIm->elemSize1();
  nvcvIm->numComponents = (unsigned char)cvIm->channels();
  nvcvIm->planar = NVCV_CHUNKY;
  nvcvIm->gpuMem = NVCV_CPU;
  nvcvIm->reserved[0] = 0;
  nvcvIm->reserved[1] = 0;
}

} // namespace estimation::nvidia::helpers