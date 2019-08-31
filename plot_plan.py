import argparse
import matplotlib.pyplot as plt
import numpy as np
import yaml

def plot_data(data, info):
    """ Data format:
    position velocity control duration
    x        v        u       delta_t
    """
    # calc elapsed time from delta_t
    t = np.cumsum(data[:, [info['delta_t_index']]], axis=0)
    data = np.append(data, t, axis=1)

    t_index = data.shape[1] - 1

    for name, index in info['smooth_dims'].items():
        plt.plot(data[:, t_index], data[:, index], label=name)

    for name, index in info['step_dims'].items():
        plt.step(data[:, t_index], data[:, index], label=name)

    plt.legend()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot planning result")
    parser.add_argument('filename', help="Data file")
    parser.add_argument('info', help="Env info yaml file")
    args = parser.parse_args()

    with open(args.info, 'r') as file:
        info = yaml.safe_load(file)

    data = np.loadtxt(args.filename, dtype='float', delimiter=' ')
    plot_data(data, info)

