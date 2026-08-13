#!/usr/bin/env python3
"""Receive text/JSON or rebuild a chunked image from the UWB receiver ESP32."""

import argparse
import base64
import binascii
import hashlib
import json
import sys
import time
from collections import deque
from pathlib import Path

import serial
from serial import SerialException


BAUD_RATE = 115200
DEFAULT_PORT = "/dev/ttyACM0"
STARTUP_DELAY_SECONDS = 2.0
IMAGE_CHUNK_BYTES = 72
MAX_IMAGE_BYTES = 10 * 1024 * 1024
IMAGE_EXTENSIONS = {"jpg", "jpeg", "png", "webp", "bmp", "gif", "tif", "tiff"}


class ImageProtocolError(ValueError):
    pass


class ImageTransfer:
    def __init__(
        self,
        transfer_id: str,
        file_size: int,
        total_chunks: int,
        sha256: str,
        extension: str,
    ) -> None:
        self.transfer_id = transfer_id
        self.file_size = file_size
        self.total_chunks = total_chunks
        self.sha256 = sha256
        self.extension = extension
        self.data = bytearray(file_size)
        self.received = bytearray(total_chunks)
        self.received_count = 0
        self.duplicate_count = 0
        self.next_progress = 10


class ImageProtocolReceiver:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        self.active: ImageTransfer | None = None
        self.completed_ids: deque[str] = deque(maxlen=32)
        self.reported_unknown_ids: set[str] = set()

    @staticmethod
    def validate_transfer_id(transfer_id: str) -> None:
        if len(transfer_id) != 8:
            raise ImageProtocolError("invalid transfer ID")
        try:
            int(transfer_id, 16)
        except ValueError as error:
            raise ImageProtocolError("invalid transfer ID") from error

    @staticmethod
    def validate_sha256(sha256: str) -> None:
        if len(sha256) != 64:
            raise ImageProtocolError("invalid SHA-256")
        try:
            int(sha256, 16)
        except ValueError as error:
            raise ImageProtocolError("invalid SHA-256") from error

    def handle(self, message: str) -> bool:
        if not message.startswith("@I,"):
            return False

        fields = message.split(",")
        try:
            if len(fields) < 2:
                raise ImageProtocolError("missing image packet type")
            if fields[1] == "S":
                self.handle_start(fields)
            elif fields[1] == "D":
                self.handle_data(fields)
            elif fields[1] == "E":
                self.handle_end(fields)
            else:
                raise ImageProtocolError("unknown image packet type")
        except (ImageProtocolError, binascii.Error) as error:
            print(f"[IMAGE ERROR] {error}", flush=True)
        return True

    def handle_start(self, fields: list[str]) -> None:
        if len(fields) != 7:
            raise ImageProtocolError("invalid START packet")

        transfer_id, size_text, chunks_text, sha256, extension = fields[2:]
        self.validate_transfer_id(transfer_id)
        self.validate_sha256(sha256)

        try:
            file_size = int(size_text)
            total_chunks = int(chunks_text)
        except ValueError as error:
            raise ImageProtocolError("invalid image size or chunk count") from error

        if not 0 < file_size <= MAX_IMAGE_BYTES:
            raise ImageProtocolError("image size is outside the allowed range")
        expected_chunks = (file_size + IMAGE_CHUNK_BYTES - 1) // IMAGE_CHUNK_BYTES
        if total_chunks != expected_chunks:
            raise ImageProtocolError("chunk count does not match image size")
        if extension not in IMAGE_EXTENSIONS:
            raise ImageProtocolError("invalid image extension")

        if transfer_id in self.completed_ids:
            return

        if self.active is not None:
            if self.active.transfer_id == transfer_id:
                same_metadata = (
                    self.active.file_size == file_size
                    and self.active.total_chunks == total_chunks
                    and self.active.sha256 == sha256
                    and self.active.extension == extension
                )
                if not same_metadata:
                    raise ImageProtocolError("START metadata changed during transfer")
                return
            print(
                f"[IMAGE ABANDONED] id={self.active.transfer_id} "
                f"received={self.active.received_count}/{self.active.total_chunks}",
                flush=True,
            )

        self.active = ImageTransfer(
            transfer_id,
            file_size,
            total_chunks,
            sha256,
            extension,
        )
        self.reported_unknown_ids.discard(transfer_id)
        print(
            "\n[IMAGE START]\n"
            f"transfer_id={transfer_id}\n"
            f"size={file_size} bytes\n"
            f"chunks={total_chunks}\n"
            f"sha256={sha256}",
            flush=True,
        )

    def handle_data(self, fields: list[str]) -> None:
        if len(fields) != 5:
            raise ImageProtocolError("invalid DATA packet")

        transfer_id, index_text, encoded_chunk = fields[2:]
        self.validate_transfer_id(transfer_id)
        if transfer_id in self.completed_ids:
            return

        if self.active is None or self.active.transfer_id != transfer_id:
            if transfer_id not in self.reported_unknown_ids:
                print(
                    f"[IMAGE WARNING] DATA received before START: id={transfer_id}",
                    flush=True,
                )
                self.reported_unknown_ids.add(transfer_id)
            return

        try:
            chunk_index = int(index_text)
        except ValueError as error:
            raise ImageProtocolError("invalid chunk index") from error
        if not 0 <= chunk_index < self.active.total_chunks:
            raise ImageProtocolError("chunk index is outside the image range")

        chunk = base64.b64decode(encoded_chunk, validate=True)
        offset = chunk_index * IMAGE_CHUNK_BYTES
        expected_length = min(IMAGE_CHUNK_BYTES, self.active.file_size - offset)
        if len(chunk) != expected_length:
            raise ImageProtocolError("decoded chunk length is invalid")

        if self.active.received[chunk_index]:
            previous = self.active.data[offset : offset + expected_length]
            if previous != chunk:
                raise ImageProtocolError("duplicate chunk contains different data")
            self.active.duplicate_count += 1
            return

        self.active.data[offset : offset + expected_length] = chunk
        self.active.received[chunk_index] = 1
        self.active.received_count += 1

        progress = (self.active.received_count * 100) // self.active.total_chunks
        if progress >= self.active.next_progress or (
            self.active.received_count == self.active.total_chunks
        ):
            print(
                f"[IMAGE RECEIVE] {self.active.received_count}/"
                f"{self.active.total_chunks} ({progress}%)",
                flush=True,
            )
            while self.active.next_progress <= progress:
                self.active.next_progress += 10

        if self.active.received_count == self.active.total_chunks:
            self.finalize_active()

    def handle_end(self, fields: list[str]) -> None:
        if len(fields) != 3:
            raise ImageProtocolError("invalid END packet")

        transfer_id = fields[2]
        self.validate_transfer_id(transfer_id)
        if transfer_id in self.completed_ids:
            return
        if self.active is None or self.active.transfer_id != transfer_id:
            return
        if self.active.received_count == self.active.total_chunks:
            self.finalize_active()
            return

        missing_count = self.active.total_chunks - self.active.received_count
        first_missing = [
            str(index)
            for index, received in enumerate(self.active.received)
            if not received
        ][:10]
        print(
            f"[IMAGE INCOMPLETE] missing={missing_count}, "
            f"first_missing={','.join(first_missing)}",
            flush=True,
        )

    def next_output_path(self, transfer: ImageTransfer) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        base_name = f"image_{transfer.transfer_id}"
        candidate = self.output_dir / f"{base_name}.{transfer.extension}"
        counter = 1
        while candidate.exists() or Path(f"{candidate}.part").exists():
            candidate = self.output_dir / (
                f"{base_name}_{counter}.{transfer.extension}"
            )
            counter += 1
        return candidate

    def finalize_active(self) -> None:
        if self.active is None:
            return

        transfer = self.active
        actual_sha256 = hashlib.sha256(transfer.data).hexdigest()
        if actual_sha256 != transfer.sha256:
            print(
                "[IMAGE ERROR] SHA-256 mismatch; image was not saved\n"
                f"expected={transfer.sha256}\nactual={actual_sha256}",
                flush=True,
            )
            self.active = None
            return

        output_path = self.next_output_path(transfer)
        partial_path = Path(f"{output_path}.part")
        try:
            with partial_path.open("xb") as output_file:
                output_file.write(transfer.data)
                output_file.flush()
            partial_path.rename(output_path)
        except Exception:
            partial_path.unlink(missing_ok=True)
            raise

        print(
            "\n[IMAGE SAVED]\n"
            f"path={output_path.resolve()}\n"
            f"size={transfer.file_size} bytes\n"
            f"duplicates={transfer.duplicate_count}\n"
            f"sha256={actual_sha256}",
            flush=True,
        )
        self.completed_ids.append(transfer.transfer_id)
        self.active = None

    def report_incomplete(self) -> None:
        if self.active is not None:
            print(
                f"[IMAGE INCOMPLETE] id={self.active.transfer_id}, "
                f"received={self.active.received_count}/"
                f"{self.active.total_chunks}",
                flush=True,
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive AI Rescue Box UWB test data over USB Serial."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument(
        "--parse-json",
        action="store_true",
        help="Also parse and display valid JSON messages",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("received_images"),
        help="Directory for completed images (default: received_images)",
    )
    return parser.parse_args()


def handle_message(message: str, parse_json: bool) -> None:
    """Display the unchanged text and optionally validate its JSON structure."""
    print(f"\n[RECEIVED]\n{message}", flush=True)

    if not parse_json:
        return

    try:
        data = json.loads(message)
    except json.JSONDecodeError:
        print("[JSON] Not a valid JSON message", flush=True)
        return

    print("[JSON PARSED]", flush=True)
    print(json.dumps(data, indent=2, ensure_ascii=False), flush=True)


def main() -> int:
    args = parse_args()
    image_receiver = ImageProtocolReceiver(args.output_dir)

    try:
        with serial.Serial(
            port=args.port,
            baudrate=BAUD_RATE,
            timeout=1.0,
        ) as connection:
            print(f"[INFO] Connected to {args.port} at {BAUD_RATE} baud", flush=True)

            # Opening a USB serial port can reset an ESP32. Ignore its boot text.
            time.sleep(STARTUP_DELAY_SECONDS)
            connection.reset_input_buffer()
            print("[INFO] Waiting for UWB data...", flush=True)

            pending = bytearray()
            while True:
                chunk = connection.readline()
                if not chunk:
                    continue

                # readline() may return a partial line on timeout. Keep it until
                # the receiver ESP32 supplies the frame-ending newline.
                pending.extend(chunk)
                if not pending.endswith(b"\n"):
                    continue

                raw_line = bytes(pending)
                pending.clear()
                payload = raw_line.rstrip(b"\r\n")
                if not payload:
                    continue

                if payload.startswith(b"@I,"):
                    try:
                        protocol_message = payload.decode("ascii")
                    except UnicodeDecodeError:
                        print("[IMAGE ERROR] protocol packet is not ASCII", flush=True)
                        continue
                    image_receiver.handle(protocol_message)
                else:
                    message = payload.decode("utf-8", errors="replace")
                    handle_message(message, args.parse_json)

    except KeyboardInterrupt:
        image_receiver.report_incomplete()
        print("\n[INFO] Stopped by user")
        return 0
    except (SerialException, OSError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
