import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--path', required = True)
parser.add_argument('--sequence', required = True)
args = parser.parse_args()

sequence_path = os.path.join(args.path, args.sequence)

# Get the number of input frames

with open(os.path.join(sequence_path, 'times.txt')) as f:
	num_frames = len(f.readlines())

# Generate matching pairs 

with open(os.path.join(sequence_path, 'prev_cur_pairs.txt'), 'w') as f:
	for i in range(num_frames - 1):
		f.write('image_0/%06d.png image_0/%06d.png \n' % (i, i + 1))

with open(os.path.join(sequence_path, 'left_right_pairs.txt'), 'w') as f:
	for i in range(num_frames):
		f.write('image_0/%06d.png image_1/%06d.png \n' % (i, i))