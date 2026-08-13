# AI Rescue Box UWB Test

ESP32와 DWM1000을 이용해 **Basecamp Laptop에서 보낸 텍스트와 사진을 Jetson Orin Nano에서 수신**하는 최소 통신 테스트 프로젝트다.

```text
Basecamp Laptop
  → USB Serial → ESP32 + DWM1000 (Sender)
  → UWB
  → ESP32 + DWM1000 (Receiver) → USB Serial
  → Jetson Orin Nano
```

ROS 2, SLAM, Camera, YOLO, 지도 처리와 GUI는 사용하지 않는다.

## 장비 구성

| 장비 | ESP32 PlatformIO 환경 | Python 프로그램 |
|---|---|---|
| Basecamp Laptop | `uwb_sender` | `jetson/sender.py` |
| Jetson Orin Nano | `uwb_receiver` | `basecamp/receiver.py` |

폴더 이름은 최초 개발 방향에서 정해졌기 때문에 현재 장비 역할과 반대로 보일 수 있다. 실제 동작은 `sender.py`가 송신, `receiver.py`가 수신이다.

## DWM1000 배선

송신부와 수신부를 똑같이 연결한다.

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

DWM1000은 반드시 `3.3V`로 연결한다. 코드의 `SPI.begin(18, 19, 23, 2)`에서 GPIO 2는 SPI 객체 설정값이며, 실제 DWM1000 CS 배선은 GPIO 5다.

## 폴더 구조

```text
UWB_Test/
├── platformio.ini
├── jetson/
│   ├── sender.py
│   └── requirements.txt
├── basecamp/
│   ├── receiver.py
│   └── requirements.txt
├── esp32/
│   ├── scripts/patch_dw1000_esp32.py
│   └── src/
│       ├── uwb_sender/main.cpp
│       └── uwb_receiver/main.cpp
└── docs/test_result.md
```

## 1. 프로젝트 받기

Laptop과 Jetson에서 각각 실행한다.

```bash
cd ~
git clone https://github.com/Jeong-Yun-Kim/UWB_Test.git
```

## 2. ESP32 펌웨어 업로드

Laptop과 Jetson의 VS Code에 `PlatformIO IDE` 확장을 설치하고 `~/UWB_Test` 폴더를 연다.

### Laptop 송신 ESP32

```text
PlatformIO → PROJECT TASKS → uwb_sender → General → Upload
```

### Jetson 수신 ESP32

```text
PlatformIO → PROJECT TASKS → uwb_receiver → General → Upload
```

두 장비 모두 기본 USB 포트는 `/dev/ttyACM0`이다. 처음 빌드할 때 ESP32 플랫폼과 DW1000 라이브러리를 내려받기 위해 인터넷 연결이 필요할 수 있다.

## 3. Python 설치

최초 한 번만 실행한다.

### Jetson

```bash
cd ~/UWB_Test/basecamp
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

### Laptop

```bash
cd ~/UWB_Test/jetson
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

## 4. 실행

항상 **Jetson 수신기를 먼저 실행**하고, `Waiting for UWB data...`가 나오면 Laptop에서 전송한다.

### Jetson: 수신

```bash
cd ~/UWB_Test/basecamp
source .venv/bin/activate
python3 receiver.py --output-dir ~/uwb_received_images
```

### Laptop: 텍스트 10개 전송

```bash
cd ~/UWB_Test/jetson
source .venv/bin/activate
python3 sender.py --count 10
```

### Laptop: 사진 전송

```bash
cd ~/UWB_Test/jetson
source .venv/bin/activate
python3 sender.py --image ~/Pictures/test.jpg
```

수신된 사진은 Jetson의 `~/uwb_received_images/`에 저장된다.

## 성공 출력

텍스트가 도착하면 Jetson에 다음처럼 출력된다.

```text
[RECEIVED]
HELLO AI RESCUE BOX 001
```

사진이 정상적으로 재조립되고 SHA-256 검증을 통과하면 다음처럼 출력된다.

```text
[IMAGE SAVED]
path=/home/user/uwb_received_images/image_1234abcd.jpg
size=...
sha256=...
```

Laptop의 `[UWB SENT]`는 송신 ESP32가 로컬 전송을 마쳤다는 뜻이며 Jetson 수신 성공 ACK는 아니다. 최종 성공 여부는 Jetson 출력으로 확인한다.

## 전송 제한

| 항목 | 제한 |
|---|---|
| 텍스트 | 한 메시지 최대 120바이트 |
| 사진 | 최대 10 MiB |
| 사진 형식 | JPG, JPEG, PNG, WEBP, BMP, GIF, TIF, TIFF |
| 사진 분할 | 원본 72바이트 단위 |
| 기본 중복 전송 | 청크당 2회 |
| 예상 사진 처리량 | 약 0.6~0.7 KiB/s |

첫 시험은 `20~50 KiB` 정도의 작은 JPEG를 권장한다. 큰 사진은 전송 시간이 매우 오래 걸린다.

## 문제 해결

### Serial Port 확인

```bash
python3 -m serial.tools.list_ports -v
```

### Serial 권한 오류

```bash
sudo usermod -aG dialout "$USER"
```

명령 실행 후 로그아웃하고 다시 로그인한다.

### Port busy

PlatformIO Serial Monitor를 닫고 점유 프로그램을 확인한다.

```bash
lsof /dev/ttyACM0
```

PlatformIO Monitor와 Python 프로그램은 `/dev/ttyACM0`을 동시에 사용할 수 없다.

### 사진이 불완전하게 수신될 때

Laptop에서 반복 횟수를 늘려 다시 전송한다.

```bash
python3 sender.py --image ~/Pictures/test.jpg --repeat 3
```

### 송신은 완료됐지만 Jetson에 출력이 없을 때

두 보드의 `3.3V`, GND, IRQ 27, CS 5, RST 15 배선과 안테나 방향을 확인하고 처음에는 1~3 m 가시거리에서 시험한다.

## 테스트 기록

실제 하드웨어 시험 결과는 [`docs/test_result.md`](docs/test_result.md)에 기록한다.
