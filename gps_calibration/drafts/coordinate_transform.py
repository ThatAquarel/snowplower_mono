import numpy as np

a = np.array([[0,0,1],[0,10,1],[10,0,1]])
b = np.array([[2,2,1],[2,12,1],[12,2,1]])

x = np.linalg.solve(a, b)
