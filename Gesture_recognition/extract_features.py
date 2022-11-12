import cv2
import numpy as np
import pandas as pd

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

std_ofs = []

i = 1
for video_file in video_files:
    video = read_video(video_file)
    video_features = []
    for y in range(1, 4):
        for x in range(1, 4):
            of = optical_flow_video(video[:,(x-1)*(video.shape[1]//3):x*(video.shape[1]//3), (y-1)*(video.shape[2]//3):y*(video.shape[2]//3), :])
            video_features.append(np.std(of, axis=(0,1,2)))
    std_ofs.append(np.reshape(video_features, (-1,))) 
    print('Video', i, 'done')
    i += 1

std_ofs = np.array(std_ofs)

df = pd.DataFrame(std_ofs)
df['target'] = labels
df.to_csv('./data.csv')