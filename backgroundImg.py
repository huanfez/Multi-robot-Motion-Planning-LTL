import numpy as np
import matplotlib.pyplot as plt
from scipy.misc import imread
import matplotlib.cbook as cbook

np.random.seed(0)
x = np.random.uniform(0.0,10.0,15)
y = np.random.uniform(0.0,10.0,15)

datafile = cbook.get_sample_data('/media/hz/New Volume/GAP/GAP_manual/MotionPlanning/RoomPlans.jpg')
img = imread(datafile)
plt.scatter(x,y,zorder=1)
plt.imshow(img, zorder=0, extent=[0.5, 8.0, 1.0, 7.0])
plt.show()
