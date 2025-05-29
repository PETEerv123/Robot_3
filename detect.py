import cv2
import numpy as np
import imutils
import time
def map_value(value, old_min, old_max, new_min, new_max):
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
# Ma trận Tcb (cho trước)
Tcb = np.array([[0, 1, 0, 320],
                [0.9659, 0 , -0.2588, 10],
                [-0.2588, 0, -0.9659, -240],
                [0, 0, 0, 1]])

# Kết quả hiệu chỉnh camera mới


mtx = np.array([[871.58348541,   0., 285.99022781],
                          [  0., 870.34601919, 255.07868665],
                          [  0.,   0.,   1.]])
dist = np.array([[-3.64036380e-01,  6.93693449e-01, -4.83672563e-03, -3.89102875e-03, -5.03818042e+00]])

# Mở camera
cap = cv2.VideoCapture(1)  # Sử dụng camera ngoài, ID = 1

# Kiểm tra xem camera có mở thành công không
if not cap.isOpened():
    print("Không thể mở camera")
    exit()

# Biến lưu trữ trạng thái
last_position = None
last_color = None
last_time = time.time()
stable_time = 0.2  # Thời gian vật phải đứng yên (giây)

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Không có khung hình, kiểm tra kết nối camera")
        break

    # Xóa méo ảnh
    h, w = frame.shape[:2]
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)

    # Cắt ảnh theo ROI để loại bỏ viền thừa
    x, y, w, h = roi
    undistorted = undistorted[y:y+h, x:x+w]

    # Chuyển sang không gian màu HSV
    hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

    # Định nghĩa các dải màu
    colors = {
    "Yellow": ([20, 100, 100], [40, 255, 255]),  # Bắt cả vàng nhạt và vàng đậm
    "Red1":   ([0, 100, 100], [10, 255, 255]),   # Vùng đỏ đầu thang
    "Red2":   ([160, 100, 100], [179, 255, 255]), # Vùng đỏ cuối thang (kết hợp với Red1 để bao hết đỏ)
    "Blue": ([90, 150, 150], [121, 255, 255]),   # Tăng giá trị Saturation và Value để lọc xanh sáng
    }
    detected_color = None
    detected_position = None

    for color, (lower, upper) in colors.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)

            # Giảm thiểu vùng cần xử lý
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Phát hiện biên và tính toán tọa độ
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in cnts:
                area = cv2.contourArea(c)
                if area > 500:  # Chỉ xử lý các đối tượng lớn
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        detected_color = color
                        detected_position = (cx, cy)

                        cv2.drawContours(undistorted, [c], -1, (0, 255, 0), 3)
                        cv2.circle(undistorted, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(
                            undistorted, f"{color}({cx}, {cy})", (cx - 20, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
                        )
                        break
            if detected_color:
                break

    if detected_color and detected_position:
            if (last_color == detected_color and last_position == detected_position):
                if time.time() - last_time >= stable_time:
                    # Tính toán tọa độq
                    cx, cy = detected_position
                    coord_matrix = np.array([[cx], [cy], [350], [1]])
                    transformed_coord = np.dot(Tcb, coord_matrix)

                    x_mapped = map_value(transformed_coord[0][0],240, 700,70,275)  #25 405
                    y_mapped = map_value(transformed_coord[1][0], -30, 350, -80, 80) # 60 445
                    
                    print(f"Vị trí pixel của vật: (cx: {cx}, cy: {cy})")
                    print(f"Màu: {detected_color}")
                    print(f"Tọa độ sau khi ánh xạ: (x: {x_mapped}, y: {y_mapped})")
                    last_time = time.time()
            else:
                last_time = time.time()

            last_color = detected_color
            last_position = detected_position
    else:
        last_color = None
        last_position = None
        last_time = time.time()

    # Hiển thị kết quả
    cv2.imshow("Undistorted and Processed", undistorted)

    if cv2.waitKey(1) & 0xFF == ord("q"):  # Bấm 'q' để thoát
        break

cap.release()
cv2.destroyAllWindows()