# 공통 설정
# 서버 주소는 코드·파일·환경 변수 어디에도 두지 않는다. 익스텐션 창의 'IP 주소' 칸에 입력한다 (익스텐션을 켤 때마다 입력).
NIM_URL = ""                                                  # 비어 있으면 화면의 IP 주소 칸에 입력한다
NIM_MODEL = "nvidia/cosmos3-nano-reasoner"
TIMEOUT_SEC = 60

# Cosmos 에 보내는 사진 형식. "jpeg" 는 PNG(약 1 MB) 보다 6~7배 작아 느린 망(Wi-Fi 등)에서 훨씬 빠르다.
# 저장하는 기록용 사진(datasets/.../images/*.png)은 그대로 PNG 로 저장한다. 예전처럼 보내려면 "png".
IMAGE_FORMAT = "jpeg"
JPEG_QUALITY = 90
