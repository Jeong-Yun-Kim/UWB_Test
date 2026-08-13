#!/usr/bin/env python3
"""Send text, JSON, or a chunked image to the UWB transmitter ESP32."""

import argparse
import base64
import hashlib
import json
import secrets
import sys
import time
from pathlib import Path

import serial
from serial import SerialException


BAUD_RATE = 115200
DEFAULT_PORT = "/dev/ttyACM0"
MAX_PAYLOAD_BYTES = 120
STARTUP_DELAY_SECONDS = 2.0
FEEDBACK_TIMEOUT_SECONDS = 1.0
IMAGE_CHUNK_BYTES = 72
MAX_IMAGE_BYTES = 10 * 1024 * 1024
IMAGE_EXTENSIONS = {"jpg", "jpeg", "png", "webp", "bmp", "gif", "tif", "tiff"}


def build_hello_message(sequence: int) -> str:
    return f"HELLO AI RESCUE BOX {sequence:03d}"


def build_json_message(_: int) -> str:
    data = {
        "robot_id": 1,
        "floor": 1,
        "x": 3.2,
        "y": 5.1,
        "status": "SEARCHING",
    }
    return json.dumps(data, separators=(",", ":"), ensure_ascii=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send AI Rescue Box test data to an ESP32 over USB Serial."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--mode",
        choices=("hello", "json"),
        default="hello",
        help="Message type (default: hello)",
    )
    source.add_argument(
        "--image",
        type=Path,
        help="Image file to send once, then exit",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between messages (default: 1.0)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of messages; 0 means run until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--image-delay",
        type=float,
        default=0.03,
        help="Delay after each image packet in seconds (default: 0.03)",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=2,
        help="Send each image data packet this many times (default: 2)",
    )
    args = parser.parse_args()

    if args.interval <= 0:
        parser.error("--interval must be greater than 0")
    if args.count < 0:
        parser.error("--count must be 0 or greater")
    if args.image_delay < 0:
        parser.error("--image-delay must be 0 or greater")
    if not 1 <= args.repeat <= 5:
        parser.error("--repeat must be between 1 and 5")
    if args.image is not None and args.count != 0:
        parser.error("--count cannot be used with --image")
    return args


def read_esp32_feedback(connection: serial.Serial) -> str | None:
    """Wait for the local DW1000 transmit result from the ESP32."""
    deadline = time.monotonic() + FEEDBACK_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if connection.in_waiting:
            raw_line = connection.readline()
            return raw_line.rstrip(b"\r\n").decode("utf-8", errors="replace")
        time.sleep(0.001)
    return None


def transmit_line(connection: serial.Serial, message: str) -> str:
    payload = message.encode("utf-8")
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError(
            f"payload is {len(payload)} bytes; maximum is {MAX_PAYLOAD_BYTES}"
        )

    connection.write(payload + b"\n")
    connection.flush()

    feedback = read_esp32_feedback(connection)
    if feedback is None:
        raise TimeoutError("ESP32 did not report UWB transmit completion")
    if feedback != "[UWB SENT]":
        raise RuntimeError(f"ESP32 reported: {feedback}")
    return feedback


def send_repeated_packet(
    connection: serial.Serial,
    packet: str,
    repeat: int,
    packet_delay: float,
) -> None:
    for _ in range(repeat):
        transmit_line(connection, packet)
        if packet_delay > 0:
            time.sleep(packet_delay)


def calculate_sha256(image_path: Path) -> str:
    digest = hashlib.sha256()
    with image_path.open("rb") as image_file:
        while block := image_file.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def get_safe_extension(image_path: Path) -> str:
    extension = image_path.suffix.lower().lstrip(".")
    if extension in IMAGE_EXTENSIONS:
        return extension
    raise ValueError(
        "unsupported image extension; use jpg, jpeg, png, webp, bmp, gif, tif, or tiff"
    )


def validate_image(image_path: Path) -> tuple[int, int, str, str]:
    if not image_path.is_file():
        raise ValueError(f"image file not found: {image_path}")

    file_size = image_path.stat().st_size
    if file_size <= 0:
        raise ValueError("image file is empty")
    if file_size > MAX_IMAGE_BYTES:
        raise ValueError(
            f"image is {file_size} bytes; maximum is {MAX_IMAGE_BYTES} bytes"
        )

    total_chunks = (file_size + IMAGE_CHUNK_BYTES - 1) // IMAGE_CHUNK_BYTES
    sha256 = calculate_sha256(image_path)
    extension = get_safe_extension(image_path)
    return file_size, total_chunks, sha256, extension


def send_image(
    connection: serial.Serial,
    image_path: Path,
    packet_delay: float,
    repeat: int,
) -> None:
    file_size, total_chunks, sha256, extension = validate_image(image_path)
    transfer_id = secrets.token_hex(4)

    start_packet = (
        f"@I,S,{transfer_id},{file_size},{total_chunks},{sha256},{extension}"
    )
    end_packet = f"@I,E,{transfer_id}"

    print(
        "\n[IMAGE SEND]\n"
        f"file={image_path}\n"
        f"size={file_size} bytes\n"
        f"chunks={total_chunks}\n"
        f"repeat={repeat}\n"
        f"packet_delay={packet_delay:.3f} seconds\n"
        f"sha256={sha256}\n"
        f"transfer_id={transfer_id}",
        flush=True,
    )

    # Control packets are cheap, so send them at least twice. The receiver
    # ignores duplicates.
    control_repeat = max(2, repeat)
    send_repeated_packet(connection, start_packet, control_repeat, packet_delay)

    next_progress = 10
    bytes_read = 0
    with image_path.open("rb") as image_file:
        for chunk_index in range(total_chunks):
            chunk = image_file.read(IMAGE_CHUNK_BYTES)
            if not chunk:
                raise ValueError("image changed while it was being sent")

            bytes_read += len(chunk)
            encoded_chunk = base64.b64encode(chunk).decode("ascii")
            data_packet = (
                f"@I,D,{transfer_id},{chunk_index},{encoded_chunk}"
            )
            send_repeated_packet(connection, data_packet, repeat, packet_delay)

            progress = ((chunk_index + 1) * 100) // total_chunks
            if progress >= next_progress or chunk_index + 1 == total_chunks:
                print(
                    f"[IMAGE SEND] {chunk_index + 1}/{total_chunks} "
                    f"({progress}%)",
                    flush=True,
                )
                while next_progress <= progress:
                    next_progress += 10

        if bytes_read != file_size or image_file.read(1):
            raise ValueError("image changed while it was being sent")

    send_repeated_packet(connection, end_packet, control_repeat, packet_delay)
    print("[IMAGE SEND COMPLETE]", flush=True)


def send_text_messages(connection: serial.Serial, args: argparse.Namespace) -> None:
    message_builder = build_hello_message if args.mode == "hello" else build_json_message

    sequence = 1
    next_send_time = time.monotonic()

    while args.count == 0 or sequence <= args.count:
        message = message_builder(sequence)
        print(f"\n[SEND]\n{message}", flush=True)
        feedback = transmit_line(connection, message)
        print(f"[ESP32]\n{feedback}", flush=True)

        sequence += 1
        next_send_time += args.interval
        remaining = next_send_time - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        else:
            next_send_time = time.monotonic()


def main() -> int:
    args = parse_args()

    try:
        with serial.Serial(
            port=args.port,
            baudrate=BAUD_RATE,
            timeout=0.25,
            write_timeout=1.0,
        ) as connection:
            print(f"[INFO] Connected to {args.port} at {BAUD_RATE} baud", flush=True)

            # Opening a USB serial port can reset an ESP32. Wait, then discard boot text.
            time.sleep(STARTUP_DELAY_SECONDS)
            connection.reset_input_buffer()

            if args.image is not None:
                send_image(connection, args.image, args.image_delay, args.repeat)
            else:
                send_text_messages(connection, args)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
        return 0
    except (SerialException, OSError, RuntimeError, TimeoutError, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
