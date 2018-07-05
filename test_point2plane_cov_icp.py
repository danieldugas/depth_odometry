import numpy as np
from matplotlib import pyplot as plt
from timeit import default_timer as timer
import os
import sys
HOME_DIR = os.path.expandvars("$HOME")
from pyniel.numpy_tools.indexing import as_idx_array

aa = np.load(os.path.join(HOME_DIR, "pepper_data/depth_clouds/scan1.npy"))
bb = np.load(os.path.join(HOME_DIR, "pepper_data/depth_clouds/scan2.npy"))
aa = aa[:,:,:3]
bb = bb[:,:,:3]
chronos = {}

# ra = np.sqrt(np.square(aa[:,:,0]) + np.square(aa[:,:,1]) + np.square(aa[:,:,2]))
# rb = np.sqrt(np.square(bb[:,:,0]) + np.square(bb[:,:,1]) + np.square(bb[:,:,2]))
plt.ion()
# plt.imshow(ra)
# plt.figure()
# plt.imshow(rb)
# plt.figure()
# plt.scatter(aa[:,:,0], aa[:,:,2], c=as_idx_array(aa[:,:,1], axis=(1)))
# plt.figure()
# plt.scatter(aa[:,:,1], aa[:,:,2], c=as_idx_array(aa[:,:,1], axis=(0)))

def normals_using_structure(aa, variance=False, k=1):
    a_ = aa[:,:,:3]
    c = a_[  k: - k,  k:  -k]
    l = a_[  k: - k,   :-2*k]
    r = a_[  k: - k,2*k:    ]
    u = a_[   :-2*k,  k:  -k]
    d = a_[2*k:    ,  k:  -k]
    n1 = np.cross(l - c, u - c, axis=-1)
    n2 = np.cross(u - c, r - c, axis=-1)
    n3 = np.cross(r - c, d - c, axis=-1)
    n4 = np.cross(d - c, l - c, axis=-1)
    nn = np.average([n1, n2, n3, n4], axis=0)
    nn = nn / np.linalg.norm(nn, axis=-1)[:,:,None]
    if variance:
        n1 = n1 / np.linalg.norm(n1, axis=-1)[:,:,None]
        n2 = n2 / np.linalg.norm(n2, axis=-1)[:,:,None]
        n3 = n3 / np.linalg.norm(n3, axis=-1)[:,:,None]
        n4 = n4 / np.linalg.norm(n4, axis=-1)[:,:,None]
        n1var = np.square(np.linalg.norm(n1 - nn, axis=-1))
        n2var = np.square(np.linalg.norm(n2 - nn, axis=-1))
        n3var = np.square(np.linalg.norm(n3 - nn, axis=-1))
        n4var = np.square(np.linalg.norm(n4 - nn, axis=-1))
    return np.pad(nn, ((k, k), (k, k), (0, 0)), 'constant', constant_values=np.nan)

chronostart = timer()
naa = normals_using_structure(aa)
nbb = normals_using_structure(bb)
chrono = timer() - chronostart
chrono_name = "computing normals with structure"
chronos[chrono_name] = chrono
print("{} : {} ms".format(chrono_name, 1000.*chrono))
naa_s = normals_using_structure(aa, k=10)
nbb_s = normals_using_structure(bb, k=10)

chronostart = timer()
aisnotnan = np.logical_not(np.any(np.isnan(naa), axis=-1))
bisnotnan = np.logical_not(np.any(np.isnan(nbb), axis=-1))
a = aa[aisnotnan]
b = bb[bisnotnan]
aidx = np.where(aisnotnan)
bidx = np.where(bisnotnan)
chrono = timer() - chronostart
chrono_name = "filtering nans"
chronos[chrono_name] = chrono
print("{} : {} ms".format(chrono_name, 1000.*chrono))

na = naa[aisnotnan]
nb = nbb[bisnotnan]
from sklearn.neighbors import KDTree
kdt = KDTree(a)
chronostart = timer()
nndist, nnidx = kdt.query(b)
nn_b_in_a = a[nnidx[:,0], :]
chrono = timer() - chronostart
chrono_name = "Nearest neighbors"
chronos[chrono_name] = chrono
print("{} : {} ms".format(chrono_name, 1000.*chrono))

normals_b_in_a = na[nnidx[:,0], :]
rotvec = np.cross(normals_b_in_a, nb, axis=-1)
from sklearn.linear_model import RANSACRegressor
ransac = RANSACRegressor()
chronostart = timer()
ransac.fit(np.zeros((len(rotvec),1)), rotvec)
bestrotvec = ransac.predict([[0]])
chrono = timer() - chronostart
chrono_name = "RANSAC"
chronos[chrono_name] = chrono
print("{} : {} ms".format(chrono_name, 1000.*chrono))


theta = np.arcsin(np.linalg.norm(rotvec, axis=-1))
avgrot = np.average(rotvec[np.logical_not(np.any(np.isnan(rotvec), axis=-1))], axis=0)

# from matplotlib import pyplot as plt; import test; plt.close("all"); test = reload(test); from test import *


plt.figure()
plt.imshow(naa)
plt.figure()
plt.imshow(nbb)
plt.figure()
plt.imshow(naa_s)
plt.figure()
plt.imshow(nbb_s)


plt.figure()
plt.imshow(naa)


