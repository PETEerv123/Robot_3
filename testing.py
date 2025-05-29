import tkinter as tk
import serial
import caculate # thư viện Kinematic
import threading
from tkinter import ttk, messagebox
import serial.tools.list_ports
import cv2
import numpy as np
import time
from PIL import Image, ImageTk  # Thêm thư viện PIL để chuyển đổi hình ảnh
L1 = 200
L2 = 212
L3 = 190


# Hàm xử lý khi nhấn nút "Lấy giá trị"
# Biến toàn cục để kiểm soát trạng thái gửi tín hiệu
ser = None
reset_time =20
def get_ports():
    """Lấy danh sách các cổng COM khả dụng."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def connect_to_arduino():
    """Kết nối tới Arduino và hiển thị thông báo."""
    global ser
    selected_port = combo_box.get()

    if selected_port:
        try:
            # Thử kết nối với cổng đã chọn
            ser = serial.Serial(selected_port, 9600, timeout=1)
            messagebox.showinfo("Kết nối thành công", f"Đã kết nối với {selected_port}!")
            # Cập nhật trạng thái kết nối
            status_label.config(text="Đã kết nối", fg="green")
        except Exception as e:
            messagebox.showerror("Lỗi kết nối", str(e))
            # Cập nhật trạng thái kết nối khi có lỗi
            status_label.config(text="Kết nối thất bại", fg="red")
    else:
        messagebox.showwarning("Chọn cổng", "Vui lòng chọn cổng COM trước!")
        status_label.config(text="Chưa kết nối", fg="red")

def send_data(data):
    """Hàm gửi dữ liệu đến Arduino."""
    global ser
    if ser is not None and ser.is_open:
        ser.write(data.encode())
        print(f"Đã gửi: {data}")
    else:
        print("Chưa kết nối tới cổng serial.")

def Inverse():
    """Tính toán Inverse Kinematics và gửi dữ liệu."""
    def worker():
        Px = entry_Px.get() or '300'
        Py = entry_Py.get() or '0'
        Pz = entry_Pz.get() or '50'
        theta1_value, _, _ = caculate.donghocnghich(Px, Py, Pz, L1, L2, L3)
        _, theta2_value, _ = caculate.donghocnghich(Px, Py, Pz, L1, L2, L3)
        _, _, theta3_value = caculate.donghocnghich(Px, Py, Pz, L1, L2, L3)

        if theta3_value > 90:
            theta3_value = 90.0

        slider1.set(theta1_value)
        slider2.set(theta2_value)
        slider3.set(theta3_value)

        entry_theta1.delete(0, tk.END)
        entry_theta1.insert(0, theta1_value)
        entry_theta2.delete(0, tk.END)
        entry_theta2.insert(0, theta2_value)
        entry_theta3.delete(0, tk.END)
        entry_theta3.insert(0, theta3_value)

        data = f"F,theta1:{theta1_value},theta2:{theta2_value},theta3:{theta3_value}"
        send_data(data)

    threading.Thread(target=worker, daemon=True).start()

def Forward():
    """Tính toán Forward Kinematics và gửi dữ liệu."""
    def worker():
        theta1_value = entry_theta1.get() or '0'
        theta2_value = entry_theta2.get() or '0'
        theta3_value = entry_theta3.get() or '0'

        Px, _, _ = caculate.donghocthuan(theta1_value, theta2_value, theta3_value, L1, L2, L3)
        _, Py, _ = caculate.donghocthuan(theta1_value, theta2_value, theta3_value, L1, L2, L3)
        _, _, Pz = caculate.donghocthuan(theta1_value, theta2_value, theta3_value, L1, L2, L3)

        slider1.set(theta1_value)
        slider2.set(theta2_value)
        slider3.set(theta3_value)

        entry_Px.delete(0, tk.END)
        entry_Px.insert(0, str(Px))
        entry_Py.delete(0, tk.END)
        entry_Py.insert(0, str(Py))
        entry_Pz.delete(0, tk.END)
        entry_Pz.insert(0, str(Pz))

        data = f"F,theta1:{theta1_value},theta2:{theta2_value},theta3:{theta3_value}"
        send_data(data)

    threading.Thread(target=worker, daemon=True).start()

def Start_btn():
    """Bắt đầu di chuyển robot."""
    threading.Thread(target=lambda: send_data('S'), daemon=True).start()

def Stop_btn():
    """Dừng robot."""
    threading.Thread(target=lambda: send_data('T'), daemon=True).start()
def Reset_btn():
    """Reset robot."""
    threading.Thread(target=lambda: send_data('R'), daemon=True).start()

def hut_btn():
    """Hút robot."""
    threading.Thread(target=lambda: send_data('H'), daemon=True).start()

def tha_btn():
    """Thả robot."""
    threading.Thread(target=lambda: send_data('L'), daemon=True).start()
def Detect():
    global sent_data
    sent_data=False

def Reset():
    """Reset giao diện."""
    entry_Px.delete(0, tk.END)
    entry_Py.delete(0, tk.END)
    entry_Pz.delete(0, tk.END)
    entry_theta1.delete(0, tk.END)
    entry_theta2.delete(0, tk.END)
    entry_theta3.delete(0, tk.END)
    slider1.set(0)
    slider2.set(0)
    slider3.set(0)
#######
def tinh_theta(Px, Py, Pz, L1, L2, L3):
    """
    Hàm này tính toán theta1, theta2, theta3 cho mỗi (Px, Py, Pz) đầu vào.
    Trả về giá trị của theta1, theta2, theta3.
    """
    theta1, _, _ = caculate.donghocnghich(Px, Py, Pz, L1, L2, L3)
    _, theta2, _ = caculate.donghocnghich(Px, Py, Pz, L1, L2, L3)
    _, _, theta3 = caculate.donghocnghich(Px, Py, Pz, L1, L2, L3)
    return theta1, theta2, theta3

def quy_hoach():
    def worker():
        Px1 = entry_Px1.get() or '300'
        Py1 = entry_Py1.get() or '0'
        Pz1 = entry_Pz1.get() or '50'
        H1  = entry_H1.get()  or '1'
        theta1_value1, theta2_value1, theta3_value1 = tinh_theta(Px1, Py1, Pz1, L1, L2, L3)
        Px2 = entry_Px2.get() or '212.17'
        Py2 = entry_Py2.get() or '-122.5'
        Pz2 = entry_Pz2.get() or '13'
        H2  = entry_H2.get()  or '0'
        theta1_value2, theta2_value2, theta3_value2 = tinh_theta(Px2, Py2, Pz2, L1, L2, L3)
        Px3 = entry_Px3.get() or '300'
        Py3 = entry_Py3.get() or '0'
        Pz3 = entry_Pz3.get() or '50'
        H3  = entry_H3.get()  or '1'
        theta1_value3, theta2_value3, theta3_value3 = tinh_theta(Px3, Py3, Pz3, L1, L2, L3)
        Px4 = entry_Px4.get() or '212.17'
        Py4 = entry_Py4.get() or '-122.5'
        Pz4 = entry_Pz4.get() or '13'
        H4  = entry_H4.get()  or '0'
        theta1_value4, theta2_value4, theta3_value4 = tinh_theta(Px4, Py4, Pz4, L1, L2, L3)
        Px5 = entry_Px5.get() or '300'
        Py5 = entry_Py5.get() or '0'
        Pz5 = entry_Pz5.get() or '50'
        H5  = entry_H5.get()  or '1'
        theta1_value5, theta2_value5, theta3_value5 = tinh_theta(Px5, Py5, Pz5, L1, L2, L3)
        Px6 = entry_Px6.get() or '212.17'
        Py6 = entry_Py6.get() or '-122.5'
        Pz6 = entry_Pz6.get() or '13'
        H6  = entry_H6.get()  or '0'
        theta1_value6, theta2_value6, theta3_value6 = tinh_theta(Px6, Py6, Pz6, L1, L2, L3)
        Px7 = entry_Px7.get() or '300'
        Py7 = entry_Py7.get() or '0'
        Pz7 = entry_Pz7.get() or '50'
        H7  = entry_H7.get()  or '1'
        theta1_value7, theta2_value7, theta3_value7 = tinh_theta(Px7, Py7, Pz7, L1, L2, L3)
        Px8 = entry_Px8.get() or '212.17'
        Py8 = entry_Py8.get() or '-122.5'
        Pz8 = entry_Pz8.get() or '13'
        H8  = entry_H8.get()  or '0'
        theta1_value8, theta2_value8, theta3_value8 = tinh_theta(Px8, Py8, Pz8, L1, L2, L3)
        Px9 = entry_Px9.get() or '300'
        Py9 = entry_Py9.get() or '0'
        Pz9 = entry_Pz9.get() or '50'
        H9  = entry_H9.get()  or '1'
        theta1_value9, theta2_value9, theta3_value9 = tinh_theta(Px9, Py9, Pz9, L1, L2, L3)
        Px10 = entry_Px10.get() or '212.17'
        Py10 = entry_Py10.get() or '-122.5'
        Pz10 = entry_Pz10.get() or '13'
        H10  = entry_H10.get()  or '0'
        theta1_value10, theta2_value10, theta3_value10 = tinh_theta(Px10, Py10, Pz10, L1, L2, L3)
        data = f"I,theta1:{theta1_value1},theta2:{theta2_value1},theta3:{theta3_value1},H:{H1},theta1:{theta1_value2},theta2:{theta2_value2},theta3:{theta3_value2},H:{H2},theta1:{theta1_value3},theta2:{theta2_value3},theta3:{theta3_value3},H:{H3},theta1:{theta1_value4},theta2:{theta2_value4},theta3:{theta3_value4},H:{H4},theta1:{theta1_value5},theta2:{theta2_value5},theta3:{theta3_value5},H:{H5},theta1:{theta1_value6},theta2:{theta2_value6},theta3:{theta3_value6},H:{H6},theta1:{theta1_value7},theta2:{theta2_value7},theta3:{theta3_value7},H:{H7},theta1:{theta1_value8},theta2:{theta2_value8},theta3:{theta3_value8},H:{H8},theta1:{theta1_value9},theta2:{theta2_value9},theta3:{theta3_value9},H:{H9},theta1:{theta1_value10},theta2:{theta2_value10},theta3:{theta3_value10},H:{H10}"

        send_data(data)

    threading.Thread(target=worker, daemon=True).start()

###
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
cap = cv2.VideoCapture(1)  

# Kiểm tra xem camera có mở thành công không
if not cap.isOpened():
    print("Không thể mở camera")
    exit()

# Tạo cửa sổ chính
GiaoDien = tk.Tk()
GiaoDien.title("Giao diện điều khiển")
GiaoDien.geometry('1680x1080')
GiaoDien.configure(bg='lightgray')

canvas = tk.Canvas(GiaoDien, width=1980, height=1080)  # Cập nhật kích thước canvas
canvas.grid(row=0, column=0, padx=10, pady=10)
canvas.configure(bg='lightgray')
# Biến lưu trữ trạng thái
last_position = None
last_color = None
last_time = time.time()
stable_time = 0.2  # Thời gian vật phải đứng yên (giây)
sent_data=True
last_send_time = 0  # Khai báo biến toàn cục

# Cập nhật hàm show_frame
def show_frame():
    global last_color, last_position, last_time,sent_data,last_send_time

    ret, frame = cap.read()
    if not ret or frame is None:
        print("Không có khung hình, kiểm tra kết nối camera")
        return

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
                    # Tính toán tọa độ
                    cx, cy = detected_position
                    coord_matrix = np.array([[cx], [cy], [350], [1]])
                    transformed_coord = np.dot(Tcb, coord_matrix)

                    x_mapped = map_value(transformed_coord[0][0],240, 700, 70 , 275)  # 25 405
                    y_mapped = map_value(transformed_coord[1][0], -30, 350, -80, 80) # 60 445
                    
                    if x_mapped < 120:
                        z_mapped = 20
                    elif 120 < x_mapped < 200:
                        z_mapped = 14
                    elif 200 < x_mapped < 280:
                        z_mapped = 11
                    # Tính toán góc (các tham số L1, L2, L3 là chiều dài của các khớp robot)
                    theta1_value_C, theta2_value_C, theta3_value_C = tinh_theta(x_mapped, y_mapped, z_mapped, L1, L2, L3)
                    print(f"Màu: {detected_color}")
                    print(f"Tọa độ sau khi ánh xạ: (x: {x_mapped}, y: {y_mapped})")

                    if not sent_data:
                        if detected_color == 'Yellow':
                         Dec = 2
                        elif detected_color == 'Red2' or detected_color == 'Red1':
                            Dec = 3
                        elif detected_color == 'Blue':
                            Dec = 1
                        data = f"D,theta1:{theta1_value_C},theta2:{theta2_value_C},theta3:{theta3_value_C},D:{Dec}"
                        send_data(data)
                        last_send_time = time.time()
                        print(f"Vị trí pixel của vật đã Gửi: (cx: {cx}, cy: {cy})")
                        print(f"Màu: {detected_color}")
                        print(f"Tọa độ sau khi ánh xạ: (x: {x_mapped}, y: {y_mapped})")
                        sent_data = True  # Đánh dấu là đã gửi dữ liệu
                    if sent_data and (time.time() - last_send_time) >= reset_time:
                        Detect()
            else:
                last_time = time.time()

            last_color = detected_color
            last_position = detected_position
    else:
        last_color = None
        last_position = None
        last_time = time.time()


    # Chuyển ảnh từ OpenCV (BGR) sang RGB (cho Tkinter)
    rgb_frame = cv2.cvtColor(undistorted, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(rgb_frame)
    imgtk = ImageTk.PhotoImage(image=img)

    # Hiển thị ảnh lên canvas
    canvas.create_image(800, 0 , anchor=tk.NW, image=imgtk)
    canvas.imgtk = imgtk  

    # Gọi lại hàm để tiếp tục cập nhật hình ảnh
    GiaoDien.after(10, show_frame)


show_frame()

# Lấy danh sách các cổng COM
ports = get_ports()
# Thiết lập font cho combo box
font_style = ('Helvetica', 12)  #
canvas.pack(pady=0, padx=0)


# Tạo combo box
combo_box = ttk.Combobox(GiaoDien, values=ports, font=font_style)
combo_box.pack(pady=20)
combo_box.set("Chọn cổng com") 
combo_box.place(x=1350, y=85)
# ketnoi
connect_button = tk.Button(GiaoDien, text="Kết nối", command=connect_to_arduino, bg='gray', fg='black', font=("Arial", 16),width=10)
connect_button.pack(pady=20)
connect_button.place(x=1380, y=200)

baud_rates = [9600, 115200, 19200, 38400, 57600] 
combo_box_baud = ttk.Combobox(GiaoDien, values=baud_rates, font=font_style)
combo_box_baud.pack(pady=10)
combo_box_baud.set("Chọn tốc độ baud")  
combo_box_baud.place(x=1350, y=125)
# Nhãn để hiển thị trạng thái kết nối
status_label = tk.Label(GiaoDien, text="Chưa kết nối", font=("Arial", 12), fg="red")
status_label.pack(pady=10)
status_label.place(x=1350, y=164)


# Tạo nút để lấy giá trị từ ma trận
quyhoachbut = tk.Button(GiaoDien, text="Quy Hoạch", command=quy_hoach, font=("Arial", 14),bg='gray', fg='black')
quyhoachbut.pack(pady=20)


result_label = tk.Label(GiaoDien, text="", font=("Arial", 14),bg='lightgray', fg='black')
result_label.pack(pady=20)

entry_theta1 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_theta1.pack(pady=20)
theta1_label = tk.Label(GiaoDien, text="Theta1:", font=("Arial", 16), bg='gray', fg='black')
theta1_label.pack(pady=(20, 0))

entry_theta2 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_theta2.pack(pady=20)
theta1_labe2 = tk.Label(GiaoDien, text="Theta2:", font=("Arial", 16),bg='gray', fg='black')
theta1_labe2.pack(pady=(20, 0))

entry_theta3 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_theta3.pack(pady=20)
theta1_labe3 = tk.Label(GiaoDien, text="Theta3:", font=("Arial", 16), bg='gray', fg='black')
theta1_labe3.pack(pady=(20, 0))
#ô nhập dữ liệu cho tọa độ 
#ô dữ liệu Px
label_Px = tk.Label(GiaoDien, text="Px:", font=("Arial", 16), bg='gray', fg='black')
label_Px.pack(pady=20)
entry_Px = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px.pack(pady=(20, 0))

#ô dữ liệu Py
label_Py = tk.Label(GiaoDien, text="Py:", font=("Arial", 16), bg='gray', fg='black')
label_Py.pack(pady=20)
entry_Py = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py.pack(pady=(20, 0))
#Ô Dữ liệu Pz
label_Pz = tk.Label(GiaoDien, text="Pz:", font=("Arial", 16), bg='gray', fg='black')
label_Pz.pack(pady=20)
entry_Pz = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz.pack(pady=(20, 0))
#tạo nút dùng để phân biệt kinematic và inver
# nút PX PY PZ 1
label_vitri1 = tk.Label(GiaoDien, text="Vi tri 1", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri1.pack(pady=20)
label_Px1 = tk.Label(GiaoDien, text="Px1:", font=("Arial", 16), bg='gray', fg='black')
label_Px1.pack(pady=20)
entry_Px1 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px1.pack(pady=(20, 0))

#ô dữ liệu Py
label_Py1 = tk.Label(GiaoDien, text="Py1:", font=("Arial", 16), bg='gray', fg='black')
label_Py1.pack(pady=20)
entry_Py1 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py1.pack(pady=(20, 0))


#Ô Dữ liệu Pz
label_Pz1 = tk.Label(GiaoDien, text="Pz1:", font=("Arial", 16), bg='gray', fg='black')
label_Pz1.pack(pady=20)
entry_Pz1 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz1.pack(pady=(20, 0))

label_H1 = tk.Label(GiaoDien, text="H1 :", font=("Arial", 16), bg='gray', fg='black')
label_H1.pack(pady=20)
entry_H1 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H1.pack(pady=(20, 0))

label_vitri2 = tk.Label(GiaoDien, text="Vi tri 2", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri2.pack(pady=20)
label_Px2 = tk.Label(GiaoDien, text="Px2:", font=("Arial", 16), bg='gray', fg='black')
label_Px2.pack(pady=20)
entry_Px2 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px2.pack(pady=(20, 0))
#ô dữ liệu Py
label_Py2 = tk.Label(GiaoDien, text="Py2:", font=("Arial", 16), bg='gray', fg='black')
label_Py2.pack(pady=20)
entry_Py2 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py2.pack(pady=(20, 0))
#Ô Dữ liệu Pz
label_Pz2 = tk.Label(GiaoDien, text="Pz2:", font=("Arial", 16), bg='gray', fg='black')
label_Pz2.pack(pady=20)
entry_Pz2 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz2.pack(pady=(20, 0))
label_H2 = tk.Label(GiaoDien, text="H2 :", font=("Arial", 16), bg='gray', fg='black')
label_H2.pack(pady=20)
entry_H2 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H2.pack(pady=(20, 0))
#
label_vitri3 = tk.Label(GiaoDien, text="Vi tri 3", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri3.pack(pady=20)
label_Px3 = tk.Label(GiaoDien, text="Px3:", font=("Arial", 16), bg='gray', fg='black')
label_Px3.pack(pady=20)
entry_Px3 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px3.pack(pady=(20, 0))

#ô dữ liệu Py
label_Py3 = tk.Label(GiaoDien, text="Py3:", font=("Arial", 16), bg='gray', fg='black')
label_Py3.pack(pady=20)
entry_Py3 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py3.pack(pady=(20, 0))
#Ô Dữ liệu Pz
label_Pz3 = tk.Label(GiaoDien, text="Pz3:", font=("Arial", 16), bg='gray', fg='black')
label_Pz3.pack(pady=20)
entry_Pz3 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz3.pack(pady=(20, 0))
label_H3 = tk.Label(GiaoDien, text="H3 :", font=("Arial", 16), bg='gray', fg='black')
label_H3.pack(pady=20)
entry_H3 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H3.pack(pady=(20, 0))
#
label_vitri4 = tk.Label(GiaoDien, text="Vi tri 4", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri4.pack(pady=20)
label_Px4 = tk.Label(GiaoDien, text="Px4:", font=("Arial", 16), bg='gray', fg='black')
label_Px4.pack(pady=20)
entry_Px4 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px4.pack(pady=(20, 0))
label_Py4 = tk.Label(GiaoDien, text="Py4:", font=("Arial", 16), bg='gray', fg='black')
label_Py4.pack(pady=20)
entry_Py4 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py4.pack(pady=(20, 0))
label_Pz4 = tk.Label(GiaoDien, text="Pz4:", font=("Arial", 16), bg='gray', fg='black')
label_Pz4.pack(pady=20)
entry_Pz4 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz4.pack(pady=(20, 0))
label_H4 = tk.Label(GiaoDien, text="H4 :", font=("Arial", 16), bg='gray', fg='black')
label_H4.pack(pady=20)
entry_H4 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H4.pack(pady=(20, 0))
#
label_vitri5 = tk.Label(GiaoDien, text="Vi tri 5", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri5.pack(pady=20)
label_Px5 = tk.Label(GiaoDien, text="Px5:", font=("Arial", 16), bg='gray', fg='black')
label_Px5.pack(pady=20)
entry_Px5 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px5.pack(pady=(20, 0))
label_Py5 = tk.Label(GiaoDien, text="Py5:", font=("Arial", 16), bg='gray', fg='black')
label_Py5.pack(pady=20)
entry_Py5 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py5.pack(pady=(20, 0))
label_Pz5 = tk.Label(GiaoDien, text="Pz5:", font=("Arial", 16), bg='gray', fg='black')
label_Pz5.pack(pady=20)
entry_Pz5 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz5.pack(pady=(20, 0))
label_H5 = tk.Label(GiaoDien, text="H5 :", font=("Arial", 16), bg='gray', fg='black')
label_H5.pack(pady=20)
entry_H5 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H5.pack(pady=(20, 0))
#
label_vitri6 = tk.Label(GiaoDien, text="Vi tri 6", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri6.pack(pady=20)
label_Px6 = tk.Label(GiaoDien, text="Px6:", font=("Arial", 16), bg='gray', fg='black')
label_Px6.pack(pady=20)
entry_Px6 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px6.pack(pady=(20, 0))
label_Py6 = tk.Label(GiaoDien, text="Py6:", font=("Arial", 16), bg='gray', fg='black')
label_Py6.pack(pady=20)
entry_Py6 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py6.pack(pady=(20, 0))
label_Pz6 = tk.Label(GiaoDien, text="Pz6:", font=("Arial", 16), bg='gray', fg='black')
label_Pz6.pack(pady=20)
entry_Pz6 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz6.pack(pady=(20, 0))
label_H6 = tk.Label(GiaoDien, text="H6 :", font=("Arial", 16), bg='gray', fg='black')
label_H6.pack(pady=20)
entry_H6 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H6.pack(pady=(20, 0))
#
label_vitri7 = tk.Label(GiaoDien, text="Vi tri 7", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri7.pack(pady=20)
label_Px7 = tk.Label(GiaoDien, text="Px7:", font=("Arial", 16), bg='gray', fg='black')
label_Px7.pack(pady=20)
entry_Px7 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px7.pack(pady=(20, 0))
label_Py7 = tk.Label(GiaoDien, text="Py7:", font=("Arial", 16), bg='gray', fg='black')
label_Py7.pack(pady=20)
entry_Py7 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py7.pack(pady=(20, 0))
label_Pz7 = tk.Label(GiaoDien, text="Pz7:", font=("Arial", 16), bg='gray', fg='black')
label_Pz7.pack(pady=20)
entry_Pz7 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz7.pack(pady=(20, 0))
label_H7 = tk.Label(GiaoDien, text="H7 :", font=("Arial", 16), bg='gray', fg='black')
label_H7.pack(pady=20)
entry_H7 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H7.pack(pady=(20, 0))
#
label_vitri8 = tk.Label(GiaoDien, text="Vi tri 8", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri8.pack(pady=20)
label_Px8 = tk.Label(GiaoDien, text="Px8:", font=("Arial", 16), bg='gray', fg='black')
label_Px8.pack(pady=20)
entry_Px8 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px8.pack(pady=(20, 0))
label_Py8 = tk.Label(GiaoDien, text="Py8:", font=("Arial", 16), bg='gray', fg='black')
label_Py8.pack(pady=20)
entry_Py8 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py8.pack(pady=(20, 0))
label_Pz8 = tk.Label(GiaoDien, text="Pz8:", font=("Arial", 16), bg='gray', fg='black')
label_Pz8.pack(pady=20)
entry_Pz8 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz8.pack(pady=(20, 0))
label_H8 = tk.Label(GiaoDien, text="H8 :", font=("Arial", 16), bg='gray', fg='black')
label_H8.pack(pady=20)
entry_H8 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H8.pack(pady=(20, 0))
#
label_vitri9 = tk.Label(GiaoDien, text="Vi tri 9", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri9.pack(pady=20)
label_Px9 = tk.Label(GiaoDien, text="Px9:", font=("Arial", 16), bg='gray', fg='black')
label_Px9.pack(pady=20)
entry_Px9 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px9.pack(pady=(20, 0))
label_Py9 = tk.Label(GiaoDien, text="Py9:", font=("Arial", 16), bg='gray', fg='black')
label_Py9.pack(pady=20)
entry_Py9 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py9.pack(pady=(20, 0))
label_Pz9 = tk.Label(GiaoDien, text="Pz9:", font=("Arial", 16), bg='gray', fg='black')
label_Pz9.pack(pady=20)
entry_Pz9 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz9.pack(pady=(20, 0))
label_H9 = tk.Label(GiaoDien, text="H9 :", font=("Arial", 16), bg='gray', fg='black')
label_H9.pack(pady=20)
entry_H9 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H9.pack(pady=(20, 0))
#
label_vitri10 = tk.Label(GiaoDien, text="Vi tri 10", font=("Arial", 16), bg='lightgray', fg='black')
label_vitri10.pack(pady=20)
label_Px10 = tk.Label(GiaoDien, text="Px10:", font=("Arial", 16), bg='gray', fg='black')
label_Px10.pack(pady=20)
entry_Px10 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Px10.pack(pady=(20, 0))
label_Py10 = tk.Label(GiaoDien, text="Py10:", font=("Arial", 16), bg='gray', fg='black')
label_Py10.pack(pady=20)
entry_Py10 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Py10.pack(pady=(20, 0))
label_Pz10 = tk.Label(GiaoDien, text="Pz10:", font=("Arial", 16), bg='gray', fg='black')
label_Pz10.pack(pady=20)
entry_Pz10 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_Pz10.pack(pady=(20, 0))
label_H10 = tk.Label(GiaoDien, text="H10 :", font=("Arial", 16), bg='gray', fg='black')
label_H10.pack(pady=20)
entry_H10 = tk.Entry(GiaoDien, font=("Arial", 16), width=8)
entry_H10.pack(pady=(20, 0))
# Tạo nút gửi
Inverse_button = tk.Button(GiaoDien, text="Inverse", command=Inverse, bg='gray', fg='black', font=("Arial", 16),width=10)
Inverse_button.pack(pady=10)
buttonForward = tk.Button(GiaoDien, text="Forward", command=Forward,bg='gray', fg='black', font=("Arial", 16),width=10)
buttonForward.pack(pady=10)

Reset_nut_button = tk.Button(GiaoDien, text="Reset", command=Reset,bg='gray', fg='black', font=("Arial", 16),width=10)
Reset_nut_button.pack(pady=10)

# Tạo nút start
start_button = tk.Button(GiaoDien, text="HOME", command=Start_btn, bg='gray', fg='black', font=("Arial", 16),width=10)
start_button.pack(pady=10)
#tạo nút stop
stop_button = tk.Button(GiaoDien, text="STOP", command=Stop_btn, bg='red', fg='black', font=("Arial", 16),width=10)
stop_button.pack(pady=10)
#
Reset_button = tk.Button(GiaoDien, text="Reset_Robot", command=Reset_btn, bg='red', fg='black', font=("Arial", 16),width=10)
Reset_button.pack(pady=10)
#hút 
hut_button = tk.Button(GiaoDien, text="Hut", command=hut_btn, bg='gray', fg='black', font=("Arial", 16),width=10)
hut_button.pack(pady=10)
#thả
tha_button = tk.Button(GiaoDien, text="Tha", command=tha_btn, bg='gray', fg='black', font=("Arial", 16),width=10)
tha_button.pack(pady=10)

detectbutton = tk.Button(GiaoDien, text="Detect", command=Detect, bg='gray', fg='black', font=("Arial", 16),width=10)
detectbutton.pack(pady=10)


entry_theta1.place(x=400  , y=50)  # Đặt tọa độ
theta1_label.place(x=300 , y=50)
#
entry_theta2.place(x=400  ,y=100)
theta1_labe2.place(x=300, y=100)
#
entry_theta3.place(x=400 ,y=150)
theta1_labe3.place(x=300, y=150)
#
entry_theta2.place(x=400   ,y=100)
theta1_labe2.place(x=300  ,y=100)
#
label_Px.place(x=50,y=50)
entry_Px.place(x=100,y=50)
#
label_Py.place(x=50,y=100)
entry_Py.place(x=100,y=100)
#
label_Pz.place(x=50,y=150)
entry_Pz.place(x=100,y=150)
#
label_vitri1.place(x=380,y=450)
label_Px1.place(x=300,y=500)
entry_Px1.place(x=360,y=500)
label_Py1.place(x=300,y=550)
entry_Py1.place(x=360,y=550)
label_Pz1.place(x=300,y=600)
entry_Pz1.place(x=360,y=600)
label_H1.place(x=300,y=650)
entry_H1.place(x=360,y=650)
#
label_vitri2.place(x=580,y=450)
label_Px2.place(x=500,y=500)
entry_Px2.place(x=560,y=500)
label_Py2.place(x=500,y=550)
entry_Py2.place(x=560,y=550)
label_Pz2.place(x=500,y=600)
entry_Pz2.place(x=560,y=600)
label_H2.place(x=500,y=650)
entry_H2.place(x=560,y=650)
#
label_vitri3.place(x=780,y=450)
label_Px3.place(x=700,y=500)
entry_Px3.place(x=760,y=500)
label_Py3.place(x=700,y=550)
entry_Py3.place(x=760,y=550)
label_Pz3.place(x=700,y=600)
entry_Pz3.place(x=760,y=600)
label_H3.place(x=700,y=650)
entry_H3.place(x=760,y=650)
#
label_vitri4.place(x=980,y=450)
label_Px4.place(x=900,y=500)
entry_Px4.place(x=960,y=500)
label_Py4.place(x=900,y=550)
entry_Py4.place(x=960,y=550)
label_Pz4.place(x=900,y=600)
entry_Pz4.place(x=960,y=600)
label_H4.place(x=900,y=650)
entry_H4.place(x=960,y=650)
#
label_vitri5.place(x=1180,y=450)
label_Px5.place(x=1100,y=500)
entry_Px5.place(x=1160,y=500)
label_Py5.place(x=1100,y=550)
entry_Py5.place(x=1160,y=550)
label_Pz5.place(x=1100,y=600)
entry_Pz5.place(x=1160,y=600)
label_H5.place(x=1100,y=650)
entry_H5.place(x=1160,y=650)
#
label_vitri6.place(x=380,y=690)
label_Px6.place(x=300,y=730)
entry_Px6.place(x=360,y=730)
label_Py6.place(x=300,y=780)
entry_Py6.place(x=360,y=780)
label_Pz6.place(x=300,y=830)
entry_Pz6.place(x=360,y=830)
label_H6.place(x=300,y=880)
entry_H6.place(x=360,y=880)
#
label_vitri7.place(x=580,y=690)
label_Px7.place(x=500,y=730)
entry_Px7.place(x=560,y=730)
label_Py7.place(x=500,y=780)
entry_Py7.place(x=560,y=780)
label_Pz7.place(x=500,y=830)
entry_Pz7.place(x=560,y=830)
label_H7.place(x=500,y=880)
entry_H7.place(x=560,y=880)
#
label_vitri8.place(x=780,y=690)
label_Px8.place(x=700,y=730)
entry_Px8.place(x=760,y=730)
label_Py8.place(x=700,y=780)
entry_Py8.place(x=760,y=780)
label_Pz8.place(x=700,y=830)
entry_Pz8.place(x=760,y=830)
label_H8.place(x=700,y=880)
entry_H8.place(x=760,y=880)
#
label_vitri9.place(x=980,y=690)
label_Px9.place(x=900,y=730)
entry_Px9.place(x=960,y=730)
label_Py9.place(x=900,y=780)
entry_Py9.place(x=960,y=780)
label_Pz9.place(x=900,y=830)
entry_Pz9.place(x=960,y=830)
label_H9.place(x=900,y=880)
entry_H9.place(x=960,y=880)
#
label_vitri10.place(x=1180,y=690)
label_Px10.place(x=1100,y=730)
entry_Px10.place(x=1160,y=730)
label_Py10.place(x=1100,y=780)
entry_Py10.place(x=1160,y=780)
label_Pz10.place(x=1100,y=830)
entry_Pz10.place(x=1160,y=830)
label_H10.place(x=1100,y=880)
entry_H10.place(x=1160,y=880)
#nút
Inverse_button.place(x=550,y=50)
#
buttonForward.place(x=550,y=100)
#start
start_button.place(x=50,y=700)
#
stop_button.place(x=50, y=650)
#
Reset_nut_button.place(x=50,y=450)
#
hut_button.place(x=50,y=600)
#
tha_button.place(x=50,y=550)
#
Reset_button.place(x=50,y=500)

detectbutton.place(x=600,y=300)
result_label.place(x= 1000 , y= 550)
quyhoachbut.place(x=1300, y = 600 )

# Tạo thanh trượt
slider1 = tk.Scale(GiaoDien, from_=-90, to=90, orient=tk.HORIZONTAL, length=200)
slider1.place(x=150, y=200)
slider2 = tk.Scale(GiaoDien, from_=-35, to=90, orient=tk.HORIZONTAL, length=200)
slider2.place(x=150, y=250)
slider3 = tk.Scale(GiaoDien, from_=-120, to=90, orient=tk.HORIZONTAL, length=200)
slider3.place(x=150, y=300)

# Bắt đầu vòng lặp chính
GiaoDien.mainloop()
cap.release()
cv2.destroyAllWindows()