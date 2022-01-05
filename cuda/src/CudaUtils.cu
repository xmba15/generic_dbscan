/**
 * @file    CudaUtils.cu
 *
 * @author  btran
 *
 */

#include <dbscan/CudaUtils.cuh>

namespace cuda
{
namespace utils
{
namespace
{
__global__ void warmUpGPUKernel()
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    ++idx;
}
}  // namespace

cudaError_t warmUpGPU()
{
    warmUpGPUKernel<<<1, 1>>>();
    cudaDeviceSynchronize();
    return cudaGetLastError();
}
}  // namespace utils
}  // namespace cuda
