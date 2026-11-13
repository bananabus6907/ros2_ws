from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# 기본 설정 (터틀봇3 Waffle Pi 기준)
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME = '/dev/ttyOpenCR' # 아까 만든 별명 사용

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 포트 열기
if not portHandler.openPort():
    print("❌ 포트를 열 수 없습니다. (런치 파일이 켜져있다면 끄고 실행하세요)")
    exit()

# 보드레이트 설정
if not portHandler.setBaudRate(BAUDRATE):
    print("❌ 보드레이트 설정 실패")
    exit()

print("🔍 모터 ID 스캔 중... (ID 1 ~ 10)")
found = False

# ID 1부터 10까지 핑(Ping) 테스트
for dxl_id in range(1, 11):
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
    if dxl_comm_result == COMM_SUCCESS:
        print(f"✅ 발견! 모터 ID: {dxl_id}")
        found = True

if not found:
    print("❌ 연결된 모터를 찾지 못했습니다. (12V 전원 스위치 및 케이블 확인 요망)")

portHandler.closePort()
