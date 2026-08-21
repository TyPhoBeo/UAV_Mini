"""Mission mẫu chạy từ REPL hoặc `import example_mission` trên board.

Dùng fc_api (wrapper BLOCKING), KHÔNG dùng module `fc` thô trực tiếp — fc.* chỉ
đẩy lệnh rồi trả về ngay, tự chờ/poll là việc của fc_api (xem fc_api.py).
"""
import fc_api as fc

fc.heartbeat.start()   # nền, không cần nếu port không có _thread (xem fc_api.Heartbeat)

fc.arm()
fc.takeoff(800)   # mm — blocking tới khi rời đất AN TOÀN rồi leo gần 800mm
fc.hover(3)
fc.move("forward", 40, 2)   # 40% biên độ, 2 giây
fc.hover(2)
fc.land()   # blocking tới khi DISARMED

fc.heartbeat.stop()
