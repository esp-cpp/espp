import cv2
import numpy as np
import os
import sys

def display(img_path):
    img_data = None
    with open(img_path, 'rb') as f:
        img_data = [int(x) for x in f.read()]
        print("Read {} bytes from {}".format(len(img_data), img_path))
    frame = cv2.imdecode(np.asarray(img_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is not None:
        while(1):
            cv2.imshow('VIDEO', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    else:
        print("No frame")

if __name__ == "__main__":
    current_directory = os.path.dirname(__file__)
    fname = current_directory+'/image.jpg'
    if len(sys.argv) == 2:
        fname = sys.argv[1]
    display(fname)
