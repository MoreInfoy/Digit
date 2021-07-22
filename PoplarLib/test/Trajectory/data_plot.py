import scipy.io as scio
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

if __name__ == '__main__':
    path_data = '/home/nimapng/Trajectory/test/data.txt'
    a = pd.read_csv(path_data)
    data_set = np.array(pd.read_csv(path_data, header=None))
    t = np.linspace(0, 6, 13)
    x = np.array(
        [0.000000,0.000000,0.000000,0.000000,0.000000,2.000000,2.000000,2.000000,0.000000,0.000000,0.000000,0.000000,0.000000])
    print(t)
    print(data_set)
    plt.figure()
    plt.plot(t, x)
    t2 = np.linspace(0, 4.99, 500)
    plt.plot(t2, data_set[0, :500])

    plt.show()
