## SmartHand - Robot Delta Phone Interaction System

### Ý tưởng
Điều khiển robot delta tương tác với app trên điện thoại iPhone (sử dụng bút cảm ứng để chạm vào màn hình điện thoại), có camera để thu hình ảnh từ điện thoại.

### Vấn đề cần giải quyết
- Cần biến tọa độ từ trong điện thoại sang tọa độ robot
- Cần biết mặt phẳng điện thoại để chạm vào an toàn
- Có các công cụ mô phỏng lại thao tác của tay người trên điện thoại

### Giải pháp (SmartHand.py)

**Phần mềm SmartHand.py** cung cấp giao diện hoàn chỉnh với các chức năng:

#### 📷 Tab Camera & Detection
1. **Camera Setup**: Kết nối và điều khiển camera
2. **Phone Detection**: 
   - Phương pháp 1: Sử dụng chessboard pattern để detect mặt phẳng
   - Phương pháp 2: Chọn thủ công 4 góc màn hình điện thoại
3. **Perspective Transform**: Biến đổi ảnh từ góc nhìn camera sang góc nhìn trực diện (top-down)

#### 🎯 Tab Calibration
1. **Coordinate Mapping**: Map tọa độ từ màn hình điện thoại sang workspace robot
   - Chọn 2 điểm reference trên màn hình
   - Đo tọa độ robot tương ứng
   - Tự động tính toán ma trận biến đổi
2. **Phone Surface Height**: Xác định độ cao Z của mặt phẳng điện thoại
3. **Save/Load Calibration**: Lưu và load dữ liệu calibration

#### 🤖 Tab Robot Control
1. **Robot Connection**: Kết nối với robot Delta qua COM port
2. **Position Display**: Hiển thị vị trí hiện tại (X, Y, Z)
3. **Basic Controls**: 
   - Home robot
   - Move to safe height
   - Emergency stop
4. **Manual Jog**: Điều khiển robot thủ công theo các trục X, Y, Z

#### 👆 Tab Touch Control
1. **Touch Settings**: 
   - Touch force (lực chạm)
   - Touch duration (thời gian chạm)
   - Movement speed
2. **Click-to-Touch Mode**: Click trực tiếp trên màn hình để robot tự động chạm
3. **Test Touch**: Test một điểm cụ thể
4. **Gesture Recording**: (Tính năng mở rộng) Ghi và phát lại các cử chỉ phức tạp

### Workflow sử dụng

```
1. Start Camera → Xem feed camera
2. Detect Phone → Chọn chessboard hoặc 4 góc màn hình
3. Calibrate Mapping → Chọn 2 điểm reference và đo tọa độ robot
4. Set Phone Z Height → Xác định độ cao mặt phẳng điện thoại
5. Connect Robot → Kết nối với robot Delta
6. Test Touch → Thử chạm một điểm để verify
7. Use! → Sẵn sàng điều khiển điện thoại
```

### Chạy chương trình

```
1. Chạy trang web để hiển thị chessboard trên điện thoại
cd chessboard
python server.py --host 0.0.0.0 --port 8080
Khi server chạy sẽ log ra địa chỉ của web. Ví dụ: http://192.168.1.7:8080
2. Mở web trên địa thoại theo địa chỉ trên.
3. Chọn bàn cờ 8x8
4. Đặt điện thoại bên dưới camera, trong vùng làm việc của robot

```

```bash
cd smartphone
pip install -r camera/requirements.txt
python SmartHand.py
```

### Tính năng an toàn
- Safe Z Height: Robot luôn di chuyển ở độ cao an toàn khi không chạm
- Visual feedback: Hiển thị tất cả các điểm trên màn hình
- Status logging: Ghi lại tất cả các hành động
- Manual control: Có thể điều khiển thủ công bất cứ lúc nào

