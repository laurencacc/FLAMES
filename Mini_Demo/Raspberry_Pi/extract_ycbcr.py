import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

frame_times = []
num_contours_list = []
avg_area_list = []
avg_Y_list = []

avg_Cr_list = []
avg_Cb_list = []
avg_diff_list = []
fire_detected_list = []

pipeline = ( 
	"libcamerasrc ! "
 	"video/x-raw,width=640,height=480,framerate=30/1,format=BGR ! "
	"videoconvert ! "
 	"appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
	print("Cannot open camera")
	exit()

prev_Y = None

while True:

	ret, frame = cap.read()
	if not ret:
		print("Can't receive frame (stream end?). Exiting...")
		break

	frame = cv2.GaussianBlur(frame, (3, 3), 0)
	frame = cv2.convertScaleAbs(frame, alpha=1.3, beta=10)

	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_hsv = np.array([0, 150, 150])
	upper_hsv = np.array([50, 255, 255])
	hsv_mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

	ycbcr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
	Y, Cr, Cb = cv2.split(ycbcr_frame)
	
	if prev_Y is not None:
		flicker =  cv2.absdiff(Y, prev_Y)
	else:
		flicker = np.zeros_like(Y)
	prev_Y = Y.copy()

	ycc_mask = (Y>180) & (Cr>140) & ((Cr-Cb) > 20) & (flicker > 10)
	ycc_mask = ycc_mask.astype(np.uint8) * 255

	combined_mask = ycc_mask.copy()

	confidence_boost = cv2.bitwise_and(ycc_mask, hsv_mask)
	combined_mask = cv2.addWeighted(combined_mask, 1.0, confidence_boost, 0.5, 0)

	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
	combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

	contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > 100:
			x, y, w, h = cv2.boundingRect(cnt)
			cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 255), 2)
			cv2.putText(frame, "FIRE DETECTED", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

	fire_areas = [cv2.contourArea(c) for c in contours if cv2.contourArea(c) > 100]
	frame_times.append(time.time())
	num_contours_list.append(len(fire_areas))
	avg_area_list.append(np.mean(fire_areas) if fire_areas else 0)
	avg_Y_list.append(np.mean(Y))
	fire_detected_list.append(1 if len(fire_areas) > 0 else 0)
	avg_Cr_list.append(np.mean(Cr))
	avg_Cb_list.append(np.mean(Cb))
	avg_diff_list = np.array(avg_Cr_list) - np.array(avg_Cb_list)

	cv2.imshow('Y (Luminance)', Y)
	cv2.imshow('Cr (Red Chroma)', Cr)
	cv2.imshow('Cb (Blue Chroma)', Cb)
	cv2.imshow('HSV Fire Mask', hsv_mask)
	cv2.imshow('YCrCb Fire Mask', ycc_mask)
	cv2.imshow('Combined Fire Mask', combined_mask)
	cv2.imshow('Fire Detection', frame)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

start = frame_times[0]
elapsed_times = [t-start for t in frame_times]

plt.figure(figsize=(12,6))

plt.subplot(2, 1, 1)
plt.plot(elapsed_times, num_contours_list, label='# Fire Regions')
plt.plot(elapsed_times, avg_area_list, label='Avg Fire Area (px^2)')
plt.legend()
plt.ylabel("Detection")
plt.grid(True)

plt.subplot (2, 1, 2)
plt.plot(elapsed_times, avg_Y_list, label='Avg Luminance (Y)', color='orange')
plt.xlabel("Time (s)")
plt.ylabel("Brightness")
plt.grid(True)

plt.figure(figsize = (12,4))
plt.plot(elapsed_times, avg_Y_list, label='Y (Brightness)', color='orange')
plt.plot(elapsed_times, avg_Cr_list, label='Cr (Red Chroma)', color='red')
plt.plot(elapsed_times, avg_Cb_list, label='Cb (Blue Chroma)', color='blue')
plt.xlabel("Time (S)")
plt.ylabel("YCbCr Values")
plt.title("YCbCr Channel Averages Over Time")
plt.legend()
plt.grid(True)

plt.figure(figsize=(12,4))
plt.plot(elapsed_times, avg_diff_list, label='Cr-Cb Difference', color='purple')
plt.plot(elapsed_times, fire_detected_list, label='Fire Detected (1/0)', color='black', linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("YCbCr / Detection Flag")
plt.title("Cr-Cb vs. Fire Detection Over Time")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
