#!/usr/bin/python3

import cv2
import numpy as np
import sys
import os
import signal
from functools import partial

def ctrlc_handler(capture, writer, sig, frame):
	capture.release()
	writer.release()
	sys.exit(os.EX_OK)

def capture_video(output_file, max_frames=None, camera_id=0):
	'''Captures a video for max_frames frames or until you press CTRL-C.
	   Saves the video in output_file
	'''
	capture = cv2.VideoCapture(camera_id)
	writer = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc('M','J','P','G'), 30, (int(capture.get(3)), int(capture.get(4))))
	signal.signal(signal.SIGINT, partial(ctrlc_handler, capture, writer))
	n_frames = 1
	print(capture.get(cv2.CAP_PROP_FPS))
	ret, frame = capture.read()
	while ret and (max_frames is None or n_frames <= max_frames):
		writer.write(frame)
		n_frames += 1
		ret, frame = capture.read()
	capture.release()
	writer.release()

if __name__ == '__main__':

	if len(sys.argv) not in [2,3,4] or not sys.argv[1].endswith('.avi'):
		print('Use:\n\t' + sys.argv[0] + 'output_file.avi [max_frames]')
		sys.exit(os.EX_USAGE)

	output_file = sys.argv[1]
	max_frames = int(sys.argv[2]) if len(sys.argv) == 3 else None

	capture_video(output_file, max_frames)
