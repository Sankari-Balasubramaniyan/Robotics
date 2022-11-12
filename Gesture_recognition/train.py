from numpy.core.fromnumeric import std
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LogisticRegression

import cv2
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
    frames = []
    capture = cv2.VideoCapture(camera_id)
    writer = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(
	    'M', 'J', 'P', 'G'), 30, (int(capture.get(3)), int(capture.get(4))))
    signal.signal(signal.SIGINT, partial(ctrlc_handler, capture, writer))
    n_frames = 1
    capture.get(cv2.CAP_PROP_FPS)
    ret, frame = capture.read()
    while ret and (max_frames is None or n_frames <= max_frames):
        writer.write(frame)
        frames.append(frame)
        n_frames += 1
        ret, frame = capture.read()
    capture.release()
    writer.release()
    return frames

def read_file_list(input_file):
    frame_list, label_list = [], []
    with open(input_file) as f:
        for l in f:
            l = l.strip().split(' ')
            frame_list.append(l[0])
            label_list.append(l[1])
    return np.array(frame_list), np.array(label_list)

def read_video(video_file):
    capture = cv2.VideoCapture(video_file)
    frames = []
    ok, frame = capture.read()
    while ok:
        frames.append(frame[...,::-1]) # let's convert frames to RGB directly
        ok, frame = capture.read()
    return np.array(frames)

def bgr2grayscale_numpy(img):
    return .0722*img[:,:,0] + .7152*img[:,:,1] + .2126*img[:,:,2]

def optical_flow_farneback(previous_frame, next_frame):
    return cv2.calcOpticalFlowFarneback(previous_frame, next_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)

def optical_flow_video(video):
    optical_flows = []
    for i in range(1, len(video)):
        previous_frame = bgr2grayscale_numpy(video[i - 1])
        current_frame = bgr2grayscale_numpy(video[i])
        optical_flows.append(optical_flow_farneback(previous_frame, current_frame))
    return np.array(optical_flows)

def optical_flow_display(flow):
    # The representation relies on the HSV color space, where colors are represented by their hue (as an angle),
    # their saturation and their intensity (value)
    flow_hsv = np.zeros(flow.shape[:2] + (3,), dtype=np.float32)
    # The hue (H) is the orientation of optical flow vectors
    flow_hsv[...,0] = np.arctan2(flow[...,1], flow[...,0])/np.pi*180. + 180.
    # Saturation (S) is just a dummy dimension here
    flow_hsv[...,1] = 1.
    # Intensity (V) is used for the norm (amplitude) of flow vectors, normalized in [0,1]
    flow_hsv[...,2] = np.linalg.norm(flow, axis=2, ord=2) # norms (amplitudes) of vectors
    flow_hsv[...,2] = (flow_hsv[...,2] - np.min(flow_hsv[...,2])) / (np.max(flow_hsv[...,2]) - np.min(flow_hsv[...,2]))
    # The final image is converted back to RGB for display 
    return cv2.cvtColor(flow_hsv, cv2.COLOR_HSV2RGB)

video_files, labels = read_file_list('videos.files')

def optical_flows_videos(video_files):
    ofs = []
    for video_file in video_files:
        video = read_video(video_file)
        ofs.append(optical_flow_video(video))
        print('Done')
    return np.array(ofs)


def create_trained_clf (data_file):
    df = pd.read_csv(data_file)
    X = df.drop(labels=['id', 'target'], axis=1)
    y = df['target']

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=1/3, random_state=42)

    clf = LogisticRegression()
    clf.fit(X_train, y_train)

    print(clf.score(X_test, y_test))

    print(clf.predict(X_test))
    print(np.array(y_test))
    return clf

def capture_gesture (clf, output_file, max_frames) :
    video = np.array(capture_video(output_file, max_frames))
    std_ofs = []
    video_features = []
    for y in range(1, 4):
        for x in range(1, 4):
            of = optical_flow_video(video[:,(x-1)*(video.shape[1]//3):x*(video.shape[1]//3), (y-1)*(video.shape[2]//3):y*(video.shape[2]//3), :])
            video_features.append(np.std(of, axis=(0,1,2)))
    std_ofs.append(np.reshape(video_features, (-1,))) 

    std_ofs = np.array(std_ofs)
    return clf.predict(std_ofs)
    
    
if __name__ == '__main__':

    if len(sys.argv) not in [2, 3, 4] or not sys.argv[1].endswith('.avi'):
        print('Use:\n\t' + sys.argv[0] + 'output_file.avi [max_frames]')
        sys.exit(os.EX_USAGE)

    output_file = sys.argv[1]
    max_frames = int(sys.argv[2]) if len(sys.argv) == 3 else None
    clf = create_trained_clf ( 'data.csv')
    
    
    while(True):
        ret = capture_gesture(clf, output_file,max_frames) 
        print('value: ' + ret)
