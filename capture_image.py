import cv2

# Specify the correct device index
device_index = '/dev/video24'

# Correct the GStreamer pipeline string
gst_str = f'v4l2src device={device_index} ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink'
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to capture image")
    exit(1)

ret, frame = cap.read()
if ret:
    cv2.imshow("Frame", frame)
    cv2.waitKey(0)
cap.release()
cv2.destroyAllWindows()
