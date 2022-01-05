/**
 * @file    CudaUtils.cuh
 *
 * @author  btran
 *
 */

#pragma once

#include <cstdio>
#include <cstdlib>

#include <cuda_runtime.h>

namespace cuda
{
namespace utils
{
cudaError_t warmUpGPU();
}  // namespace utils
}  // namespace cuda

static void HandleError(cudaError_t err, const char* file, int line)
{
    if (err != cudaSuccess) {
        printf("%s in %s at line %d\n", cudaGetErrorString(err), file, line);
        exit(EXIT_FAILURE);
    }
}

#define HANDLE_ERROR(err) (HandleError(err, __FILE__, __LINE__))
