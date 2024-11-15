import numpy as np
from scipy.linalg import block_diag

np.set_printoptions(suppress=True, precision=6)

# Q = np.diag([200, 200, 20, 0.001])

# R = np.diag([0.001, 0.001])

# # Stack Q N+1 times
# N = 1

# # Stack Q N+1 times diagonally
# Q_stacked = block_diag(*([Q] * (N+1)))

# nx = 3
# pred_step = 4

# # Calculate the range limit
# range_limit = 4

# # Generate and print the range
# for i in range(range_limit):
#     print(np.zeros((3, i*3)))

        
nx = 3
nu = 2
N = 4

# A_eq = np.zeros((0, nx*(N+1) + nu*N))
# B_eq = np.zeros((nx*(N), 1))

# print(A_eq.shape)
# arr = []
# for i in range(N):
    
#     A_d = np.identity(nx)*2

#     B_d = np.matrix([[1, 0], [1, 0], [0, 0]])
    
#     A_temp = np.hstack((np.zeros((nx, i*nx)), A_d, np.identity(nx), np.zeros((nx, nx*(N-i-1)))))
#     B_temp = np.hstack((i*np.zeros((nx, i*nu)), B_d, np.zeros((nx, nu*(N-i-1)))))
#     A_eq_temp = np.hstack((A_temp, B_temp))
#     A_eq = np.vstack((A_eq, A_eq_temp))

# print(A_eq)


G = np.matrix([[1, 0, 1, 0], 
                [1, 0, 1, 0], 
                [0, 1, 0, 1], 
                [0, 1, 0, 1]])

h = np.matrix([[1], 
               [-1], 
               [2], 
               [-2]])

A = np.zeros((0, (N+1)*nx + N*nu))

B = np.zeros((0, 1))

for i in range(N-1):

    A_temp = np.hstack((np.zeros([G.shape[0] , nx*(N+1)]), np.zeros([G.shape[0],i*nu]),  G, np.zeros([G.shape[0], N*nu - G.shape[1] - i*nu])))
    
    A = np.vstack((A, A_temp))

    B = np.vstack((B, h))

print(B)