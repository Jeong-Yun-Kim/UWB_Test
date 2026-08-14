# AI Rescue Box UWB Bidirectional Test

ESP32와 DWM1000 두 세트로 Basecamp Laptop과 Jetson Orin Nano 사이에서 **텍스트와 사진을 양방향 전송**하는 최소 시험 프로젝트다.

```text
Basecamp Laptop ↔ USB Serial ↔ ESP32 + DWM1000
                                      ↕ UWB
Jetson Orin Nano ↔ USB Serial ↔ ESP32 + DWM1000
```

DWM1000은 동시에 송수신하지 않는 **반이중(half-duplex)** 방식이다. 각 패킷은 상대편 ACK를 확인하며, ACK가 없으면 자동 재전송한다. ROS 2, SLAM, 카메라, AI, 지도 처리 및 GUI는 포함하지 않는다.

## 구성

| 장비 | PlatformIO 환경 | 실행 프로그램 |
|---|---|---|
| Basecamp Laptop | `laptop_node` | `host/uwb_node.py` |
| Jetson Orin Nano | `jetson_node` | `host/uwb_node.py` |

두 장비는 같은 Python 프로그램을 사용한다. ESP32 펌웨어의 노드 ID만 서로 다르다.

```text
UWB_Test/
├── platformio.ini
├── host/
│   └── uwb_node.py
├── esp32/
│   ├── scripts/
│   └── src/uwb_node/main.cpp
└── docs/test_result.md
```

## 배선

Laptop 측과 Jetson 측을 동일하게 연결한다.

| DWM1000 | ESP32 |
|---|---:|
| VCC / 3V3 | 3V3 |
| GND | GND |
| SCK | GPIO 18 |
| MISO | GPIO 19 |
| MOSI | GPIO 23 |
| CS / SPICSn | GPIO 5 |
| IRQ | GPIO 27 |
| RST / RSTn | GPIO 15 |

DWM1000은 반드시 `3.3V`로 연결한다. `SPI.begin(18, 19, 23, 2)`의 GPIO 2는 SPI 객체 설정값이며 실제 CS는 GPIO 5다.

## 1. 프로젝트 받기

Laptop과 Jetson에서 각각 한 번 실행한다.

```bash
cd ~
git clone https://github.com/Jeong-Yun-Kim/UWB_Test.git
cd ~/UWB_Test
```

이미 받은 프로젝트는 다음 명령으로 갱신한다.

```bash
cd ~/UWB_Test
git switch main
git pull --ff-only origin main
```

## 2. ESP32 업로드

VS Code에서 `~/UWB_Test`를 열고 PlatformIO IDE 확장을 설치한다. `Upload`는 컴파일과 업로드를 함께 수행하며, 완료 후 ESP32가 자동 실행된다.

### Laptop

```text
PlatformIO → PROJECT TASKS → laptop_node → General → Upload
```

또는 CLI:

```bash
~/.platformio/penv/bin/pio run -e laptop_node -t upload --upload-port /dev/ttyACM0
```

### Jetson

```text
PlatformIO → PROJECT TASKS → jetson_node → General → Upload
```

또는 CLI:

```bash
~/.platformio/penv/bin/pio run -e jetson_node -t upload --upload-port /dev/ttyACM0
```

Python 또는 ROS2 UWB Bridge 실행 전에는 PlatformIO Serial Monitor를 닫는다.

## 3. Python 설치

Laptop과 Jetson에서 각각 한 번 실행한다.

```bash
cd ~/UWB_Test
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r host/requirements.txt
```

## 4. 실행

양쪽 장비에서 각각 실행한다. 기본 포트는 `/dev/ttyACM0`이다.

### Laptop

```bash
cd ~/UWB_Test
source .venv/bin/activate
python3 host/uwb_node.py --name laptop --output-dir ~/uwb_received_images
```

### Jetson

```bash
cd ~/UWB_Test
source .venv/bin/activate
python3 host/uwb_node.py --name jetson --output-dir ~/uwb_received_images
```

프롬프트에서 필요한 명령만 입력한다.

```text
text HELLO AI RESCUE BOX
image /home/user/Pictures/test.jpg
quit
```

`text`와 `image`는 Laptop과 Jetson 어느 쪽에서도 사용할 수 있다. 지정한 `~/uwb_received_images` 폴더가 없으면 프로그램이 자동 생성한다.

양쪽 프로그램에 `[INFO] Connected`가 표시된 뒤 전송을 시작한다. 상대 Python 프로그램이 꺼져 있어도 상대 ESP32 자체는 무선 ACK를 보낼 수 있으므로, 최종 텍스트와 사진 도착 여부는 항상 수신 측 출력으로 확인한다.

한 건만 보내고 종료하려면 다음처럼 실행한다.

```bash
python3 host/uwb_node.py --name laptop --text "HELLO AI RESCUE BOX"
python3 host/uwb_node.py --name laptop --image ~/Pictures/test.jpg
```

다른 포트를 사용할 때만 `--port`를 지정한다.

```bash
python3 host/uwb_node.py --name laptop --port /dev/ttyACM1
```

## 성공 확인

수신 측에는 원문 또는 저장된 사진 경로가 출력된다. 송신 측의 다음 출력은 **상대편 ESP32가 패킷을 실제 수신해 응답했다는 뜻**이다.

```text
[UWB ACK]
```

사진 송신 측의 `[IMAGE SEND COMPLETE]`는 모든 조각이 무선 ACK를 받았다는 뜻이다. 파일 재조립과 SHA-256 검증까지 성공했는지는 반드시 수신 측의 `[IMAGE SAVED]`로 확인한다.

## 통신 설정과 주의점

- USB Serial: `460800 baud`
- UWB: `DW1000.MODE_LONGDATA_RANGE_LOWPOWER`
- 무선 데이터율: `110 kbps`, 긴 프리앰블
- 채널: `5`
- ACK timeout: `200 ms`
- ACK가 없으면 최대 4회 재전송
- 텍스트/상위 프로토콜 한 프레임 최대 payload: `111 bytes`

현재 펌웨어는 실제 동일 하드웨어에서 통신이 확인된 legacy `uwb_sender/uwb_receiver`의 UWB 모드와 permanent receive 동작을 기반으로 한다. 고속 모드 최적화는 기본 양방향 ACK 통신이 안정적으로 확인된 뒤 별도로 진행한다.

| 항목 | 현재 제한 |
|---|---|
| 텍스트 | UTF-8 기준 한 메시지 최대 111바이트 |
| 사진 | 최대 10 MiB |
| 사진 형식 | JPG, JPEG, PNG, WEBP, BMP, GIF, TIF, TIFF |

## AI Rescue Box 데이터 운용 권장

건물 구조도 전체에 위치와 위험 표시를 합성해 매번 사진으로 보내면 느리고 손실 복구 비용이 크다.

- 건물 베이스맵은 임무 시작 또는 버전 변경 시 한 번만 보낸다.
- 로봇 위치, 이동 경로, 요구조자 및 위험지역은 작은 좌표/상태 메시지로 보낸다.
- Basecamp는 받은 좌표를 저장된 베이스맵 위에 표시한다.
- 현장 사진은 필요할 때만 작은 JPEG/WebP 썸네일로 보낸다.
- 긴급정보와 위치정보가 지도나 사진보다 먼저 전송되도록 한다.

현재 최소 시험 코드는 우선순위 큐를 구현하지 않는다. 한 터미널에서 사진을 보내는 동안 그 터미널의 새 `text` 명령 입력은 사진 전송이 끝날 때까지 기다린다.

## 문제 해결

포트 확인:

```bash
python3 -m serial.tools.list_ports -v
```

권한 오류:

```bash
sudo usermod -aG dialout "$USER"
```

명령 실행 후 로그아웃하고 다시 로그인한다. 포트가 사용 중이면 PlatformIO Serial Monitor와 다른 송수신 프로그램을 닫고 확인한다.

```bash
lsof /dev/ttyACM0
```

ACK가 계속 오지 않으면 양쪽 펌웨어 역할, 3.3V/GND, IRQ 27, CS 5, RST 15, 안테나 방향과 양쪽 UWB 모드 일치를 확인한다.

시험 결과는 [`docs/test_result.md`](docs/test_result.md)에 기록한다.
