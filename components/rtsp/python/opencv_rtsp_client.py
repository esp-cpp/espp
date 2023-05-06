import sys
import cv2

def stream(addr, port):
    uri = f"rtsp://{addr}:{port}/mjpeg/1"
    print(f"Opening URI: {uri}, press 'q' to quit")
    vcap = cv2.VideoCapture(uri)
    while(1):
        ret, frame = vcap.read()
        cv2.imshow('VIDEO', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python ./opencv_rtsp_client <address> <rtsp_port>")
        sys.exit(1)
    stream(sys.argv[1], sys.argv[2])
