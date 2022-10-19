#pragma once

/**
 * @brief Exposes Eigen::Affine3d type to be useable with Python.
 * 
 * Here are available functions
 * 
 * frame = Affine3d()
 * frame = Affine3d.from_matrix(numpy 4x4 matrix)
 * frame.R: rotation matrix (read/write)
 * frame.t: translation part (read/write)
 * frame.mat: 4x4 matrix (read/write)
 * frame.inv: transformation inverse
 */
void exposeAffine3d();