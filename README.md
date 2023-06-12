## Project Đếm bước chân STM32 và MPU6050

### Giới thiệu
  Đây là một dự án đơn giản để đếm số bước chân sử dụng vi điều khiển STM32 và cảm biến MPU6050. Dự án này giúp theo dõi hoạt động di chuyển và đếm số bước chân của người dùng.

### Chức năng
- Đếm số bước chân: Dự án sử dụng cảm biến MPU6050 để đo gia tốc và tính toán số bước chân dựa trên dữ liệu gia tốc thu được.
- Hiển thị kết quả: Kết quả số bước chân sẽ được hiển thị trên màn hình LCD1602A.
Các thành phần chính
- Vi điều khiển STM32: Sử dụng STM32 để điều khiển toàn bộ quá trình đếm và hiển thị.
- MPU6050: Cảm biến gia tốc và gia tốc góc để đo di chuyển và tính toán số bước chân.
- Màn hình LCD1602A: Dùng để hiển thị kết quả số bước chân.

### Cách sử dụng
- Kết nối mạch: Kết nối vi điều khiển STM32 với MPU6050 và màn hình LCD1602A theo sơ đồ kết nối được cung cấp.
- Nạp chương trình: Sử dụng phần mềm KeilC V5 để nạp chương trình vào vi điều khiển STM32.
- Chạy chương trình: Sau khi nạp chương trình thành công, hệ thống sẽ bắt đầu đếm số bước chân và hiển thị kết quả trên màn hình LCD1602A.
- 
### Yêu cầu phần cứng
- Vi điều khiển STM32
- Cảm biến MPU6050
- Màn hình LCD1602A
- Các linh kiện điện tử và kết nối phụ thuộc vào sơ đồ kết nối

### Tài liệu tham khảo
- Datasheet và hướng dẫn sử dụng của STM32 và MPU6050.
- Tài liệu và ví dụ từ các nguồn tài nguyên trực tuyến và diễn đàn.
