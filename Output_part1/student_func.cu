#include "reference_calc.cpp"
#include "utils.h"
#include <stdio.h>


__global__
void rgba_to_greyscale(const uchar4* const rgbaImage,
                       unsigned char* const greyImage,
                       int numRows, int numCols)
{
        int i= (blockIdx.x*blockDim.x) + threadIdx.x; //x coordinate 
        int j= (blockIdx.y*blockDim.y) + threadIdx.y; //y cordinate
		int index=i+ j*numRows;
        uchar4 rgba = rgbaImage[index];//rgba values of the pixel to be processed 
        float value= (.299f*rgba.x) + (.587f*rgba.y) + (.114f*rgba.z); //formula
        greyImage[index] = (unsigned char)value; //storing the output value to the corresponding pixel of the grey image

}

void your_rgba_to_greyscale(const uchar4 * const h_rgbaImage, uchar4 * const d_rgbaImage,
                            unsigned char* const d_greyImage, size_t numRows, size_t numCols)
{
  int block_x= 32; // block size should always be a multiple of 32, because kernels issue instructions in warps (32 threads)
  int block_y= 1024/block_x;   // Assuming maximum active tread count 1024
  //block_y=((numCols/block_x)-1)<32?((numCols/block_x)-1):32;//**This was the best case which is runnng on 0.028736 but could not garantee for every case 
  const dim3 threadsPerBlock(block_x, block_y);  //assgin dimenctions for block
  const dim3 numBlocks( (numRows/block_x)+1, (numCols/block_y)+1); //assgin dimenctions for grid 
  rgba_to_greyscale<<<numBlocks, threadsPerBlock>>>(d_rgbaImage, d_greyImage, numRows, numCols);//kernel function call  
  cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());
}