import numpy as np
from matplotlib import pyplot as plt
import os
import sys
HOME_DIR = os.path.expandvars("$HOME")
from pyniel.numpy_tools.indexing import as_idx_array

a = np.load(os.path.join(HOME_DIR, "pepper_data/depth_clouds/scan1.npy"))
b = np.load(os.path.join(HOME_DIR, "pepper_data/depth_clouds/scan2.npy"))

ra = np.sqrt(np.square(a[:,:,0]) + np.square(a[:,:,1]) + np.square(a[:,:,2]))
rb = np.sqrt(np.square(b[:,:,0]) + np.square(b[:,:,1]) + np.square(b[:,:,2]))
plt.ion()
# plt.imshow(ra)
# plt.figure()
# plt.imshow(rb)
# plt.figure()
# plt.scatter(a[:,:,0], a[:,:,2], c=as_idx_array(a[:,:,1], axis=(1)))
# plt.figure()
# plt.scatter(a[:,:,1], a[:,:,2], c=as_idx_array(a[:,:,1], axis=(0)))

a = a[:,:,:3]
c = a[1:-1,1:-1]
l = a[1:-1, :-2]
r = a[1:-1,2:  ]
u = a[ :-2,1:-1]
d = a[2:  ,1:-1]
n1 = np.cross(l - c, u - c, axis=-1)
n2 = np.cross(u - c, r - c, axis=-1)
n3 = np.cross(r - c, d - c, axis=-1)
n4 = np.cross(d - c, l - c, axis=-1)
n1 = n1 / np.linalg.norm(n1, axis=-1)[:,:,None]
n2 = n2 / np.linalg.norm(n2, axis=-1)[:,:,None]
n3 = n3 / np.linalg.norm(n3, axis=-1)[:,:,None]
n4 = n4 / np.linalg.norm(n4, axis=-1)[:,:,None]
nn = np.average([n1, n2, n3, n4], axis=0)
n1var = np.square(np.linalg.norm(n1 - nn, axis=-1))
n2var = np.square(np.linalg.norm(n2 - nn, axis=-1))
n3var = np.square(np.linalg.norm(n3 - nn, axis=-1))
n4var = np.square(np.linalg.norm(n4 - nn, axis=-1))

plt.figure()
nidx = as_idx_array(n1var).flatten()
plt.plot(nidx, n1var.flatten())
plt.plot(nidx, n2var.flatten())
plt.plot(nidx, n3var.flatten())
plt.plot(nidx, n4var.flatten())

