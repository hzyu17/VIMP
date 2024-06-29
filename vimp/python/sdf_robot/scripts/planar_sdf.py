## Planar sdf definition using jax for computing the Hessian
# Hongzhe Yu, 12/22/2023

import jax.numpy as jnp

class Sdf2D():
    def __init__(self, origin, width, height, cell_size) -> None:
        self.origin = origin
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.field = jnp.zeros((height, width), dtype=jnp.float64)
        
    def signed_distance(self, point):
        index = self.point2index(point)
        return self.signed_distance(index)
    
    def gradient_signed_distance(self, point):
        index = self.point2index(point)
        return self.signed_distance(index)
        
    def point2index(self, point):
        index = jnp.zeros(2, dtype=jnp.float64)
        index[0] = (point[0] - self.origin[0]) / self.cell_size
        index[1] = (point[1] - self.origin[1]) / self.cell_size
        return index
    
    def signed_distance(self, index):
        return self.field[index[0], index[0]]

# def _point2index(origin, cell_size, width, height, point):
    