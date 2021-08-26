#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import math
import argparse

if __name__ == '__main__':
	
	parser = argparse.ArgumentParser()
	parser.add_argument('--pred', required = True)
	parser.add_argument('--gt', required = True)
	args = parser.parse_args()

	poses_est = np.loadtxt(args.pred)
	x_est = poses_est[:,3]
	z_est = poses_est[:,11]

	poses_gt = np.loadtxt(args.gt)
	x_gt = poses_gt[:,3]
	z_gt = poses_gt[:,11]

	plt.plot(x_est, z_est, 'b', label='Estimated trajectory')
	plt.plot(x_gt, z_gt, 'r', label='Ground truth trajectory')
	plt.axis('equal')
	plt.grid()
	plt.legend()
	plt.show()