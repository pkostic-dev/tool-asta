#!/usr/bin/env python3

# Script for testing.

import numpy as np
import tf

# real=array([ 150.98561 ,  126.700485, 1153.574   ], dtype=float32)

real = np.array(
    [150.98561 ,126.700485, 1153.574],
    dtype = np.float32
)
neg_real = -real
div_real = neg_real/1000.0
list_real = div_real.tolist()

print("real =", real)
print("neg_real =", neg_real)
print("div_real =", div_real)
print("list_real =", list_real)

print()

# orientation=array(
#    [
#       [ 0.99246067,  0.07711688,  0.09526159],
#       [-0.07708886,  0.9970163 , -0.0039798 ],
#       [-0.09528426, -0.00339381,  0.9954443 ]	
#    ], dtype=float32)

orientation = np.array(
   [
      [ 0.99246067,  0.07711688,  0.09526159],
      [-0.07708886,  0.9970163 , -0.0039798 ],
      [-0.09528426, -0.00339381,  0.9954443 ]
   ], dtype=np.float32)
flat_orientation = orientation.flatten()

print("orientation =", orientation)
print("flat_orientation =", flat_orientation)

matrix = np.mat([
    [
        flat_orientation[0],
        flat_orientation[1],
        flat_orientation[2]
    ],
    [
        flat_orientation[3],
        flat_orientation[4],
        flat_orientation[5]
    ],
    [
        flat_orientation[6],
        flat_orientation[7],
        flat_orientation[8]
    ]
])

euler_rotation = tf.transformations.euler_from_matrix(matrix,"rxyz")

print("matrix =", matrix)
print("rotation =", euler_rotation)

euler_rotations = np.zeros([20, 3])
euler_rotations[0, :] = euler_rotation

print(type(euler_rotations))
print(euler_rotations[0, 0])