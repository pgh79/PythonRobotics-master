import numpy as np
import scipy.spatial
import matplotlib.pylab as plt

a = np.array([0, 1, 1, 0, 0, 2, 0])
b = np.array([0, 0, 1, 1, 0, 1, 0])

# c = np.vstack([a, b]).T
# d = zip(a, b)#zip返回的是指向这个数组对象的指针，但是并不是数组格式了,都变成小括号()括起来的了，而不是中括号了
# print(c)
# print(*d)
# vor = scipy.spatial.Voronoi(c)
# sample_x = [ix for [ix, iy] in vor.vertices]
# sample_y = [iy for [ix, iy] in vor.vertices]
# print(sample_x, sample_y)
# plt.plot(a,b)
# plt.plot(sample_x,sample_y,'r.')
aa = [0,1,2,3,4,5,6]
print(a)
print(aa)
cc=aa*2
dd=[x*2 for x in aa]
bb=np.array(aa)*2
for i in range(len(dd)):
    dd[i]=aa[i]
print(bb)
print(cc)
print(dd)
# a=np.array([0,1,2,3])
# b=np.array([5,6,8,9])
#
# print(a)
# print(a*5)
# print(b)
# print(b*5)
#
# plt.plot(a,b)
# plt.plot(a*5,b*5)
# plt.plot(a*25,b*25,".b")
