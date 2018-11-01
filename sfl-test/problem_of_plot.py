# from __future__ import __
import matplotlib

print("matplotlib.get_beckend():" + matplotlib.get_backend())

import matplotlib.pyplot as plt


def main_test():
    plt.plot([1, 2, 3])
    plt.show()


if __name__ == '__main__':
    main_test()
    print('test')
