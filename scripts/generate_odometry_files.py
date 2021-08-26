import os
import argparse
import numpy as np
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--path', required = True)
parser.add_argument('--sequence', required = True)
parser.add_argument('--min_conf', default = 0.2)
args = parser.parse_args()

sequence_path = os.path.join(args.path, args.sequence)

# Get the number of input frames

with open(os.path.join(sequence_path, 'times.txt')) as f:
	num_frames = len(f.readlines())

# Read and check all data

lr_matches = []
for i in range(num_frames):
	lr_match_path = os.path.join(sequence_path, 'dump_left_right_matches', '%06d_%06d_matches.npz' % (i, i))
	with np.load(lr_match_path) as matches:
		lr_matches.append(dict(matches))

pc_matches = []
for i in range(num_frames - 1):
	pc_match_path = os.path.join(sequence_path, 'dump_prev_cur_matches', '%06d_%06d_matches.npz' % (i, i + 1))
	with np.load(pc_match_path) as matches:
		pc_matches.append(dict(matches))

print('Done with data loading: {}, {}'.format(len(lr_matches), len(pc_matches)))

for i in range(num_frames - 2):
	assert(np.sum(pc_matches[i]['keypoints0'] == lr_matches[i]['keypoints0']) == pc_matches[i]['keypoints0'].size)
	assert(np.sum(pc_matches[i + 1]['keypoints0'] == pc_matches[i]['keypoints1']) == pc_matches[i]['keypoints1'].size)

print('Done with data checking')

# Generate keypoints files

output_dir = Path(os.path.join(sequence_path, 'odometry'))
output_dir.mkdir(exist_ok=True, parents=True)

for i in range(num_frames):

	with open(os.path.join(output_dir, 'kp_0_%06d.txt' % i), 'w') as f:
		for kp in lr_matches[i]['keypoints0']:
			f.write('%f %f \n' % (kp[0], kp[1]))


	with open(os.path.join(output_dir, 'kp_1_%06d.txt' % i), 'w') as f:
		for kp in lr_matches[i]['keypoints1']:
			f.write('%f %f \n' % (kp[0], kp[1]))

print('Done with generating keypoints files')

# Generate flow files

with open(os.path.join(output_dir, 'flow_000000.txt'), 'w') as f:
	for kp in pc_matches[0]['keypoints0']:
		f.write('65535 \n')

c = []

for i in range(num_frames - 1):
	with open(os.path.join(output_dir, 'flow_%06d.txt' % (i + 1)), 'w') as f:
		for match, conf in zip(pc_matches[i]['matches'], pc_matches[i]['match_confidence']):
			c.append(conf)
			if (match < 0 or conf < args.min_conf):
				f.write('65535 \n')
			else:
				f.write('%d \n' % match)

print('Min conf: ', min(c))
print('Max conf: ', max(c))

print('Done with generating flow files')

# Generate stereo files

for i in range(num_frames):
	with open(os.path.join(output_dir, 'stereo_%06d.txt' % i), 'w') as f:
		for match, conf in zip(lr_matches[i]['matches'], lr_matches[i]['match_confidence']):
			if (match < 0 or conf < args.min_conf):
				f.write('65535 \n')
			else:
				f.write('%d \n' % match)

print('Done with generating stereo files')