from matplotlib import pyplot as plt
import numpy as np

data = np.loadtxt("../cmake-build-debug/data.txt")

print(data)
# plt.plot(data)
# plt.plot(data[0, :])
# plt.plot(data[1, :])
# plt.plot(data[2, :])
plt.plot(data[::2])
plt.plot(data[1::2])
plt.show()