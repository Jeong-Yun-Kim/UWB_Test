#!/usr/bin/env python3
"""Interactive bidirectional text and image host for the ESP32 UWB bridge."""

from __future__ import annotations

import argparse
import base64
import binascii
import hashlib
import queue
import re
import secrets
import shlex
import sys
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator

import serial
from serial import SerialException


DEFAULT_PORT = "/dev/ttyACM0"
BAUD_RATE = 460800
MAX_PAYLOAD_BYTES = 111
STARTUP_DELAY_SECONDS = 2.0
ACK_TIMEOUT_SECONDS = 2.0
IMAGE_CHUNK_BYTES = 66
MAX_IMAGE_BYTES = 10 * 1024 * 1024
DEFAULT_IMAGE_DELAY_SECONDS = 0.002
DEFAULT_OUTPUT_DIR = Path.home() / "uwb_received_images"
IMAGE_EXTENSIONS = {"jpg", "jpeg", "png", "webp", "bmp", "gif", "tif", "tiff"}
TRANSFER_ID_PATTERN = re.compile(r"[0-9a-f]{8}\Z")
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")


class UwbNodeError(Exception):
    """Base error for host-side protocol failures."""


class ImageProtocolError(UwbNodeError):
    """Raised when an incoming or outgoing image packet is invalid."""


@dataclass(frozen=True)
class ImageMetadata:
    path: Path
    transfer_id: str
    file_size: int
    total_chunks: int
    sha256: str
    extension: str


@dataclass
class IncomingImage:
    transfer_id: str
    file_size: int
    total_chunks: int
    sha256: str
    extension: str

    def __post_init__(self) -> None:
        self.data = bytearray(self.file_size)
        self.received = bytearray(self.total_chunks)
        self.received_count = 0
        self.duplicate_count = 0
        self.next_progress = 10


def validate_payload(payload: bytes) -> None:
    if not payload:
        raise UwbNodeError("empty messages are not allowed")
    if b"\n" in payload or b"\r" in payload:
        raise UwbNodeError("a message must contain exactly one line")
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise UwbNodeError(
            f"message is {len(payload)} bytes; maximum is {MAX_PAYLOAD_BYTES} bytes"
        )


def calculate_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while block := source.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def prepare_image(path: Path, transfer_id: str | None = None) -> ImageMetadata:
    path = path.expanduser()
    if not path.is_file():
        raise ImageProtocolError(f"image file not found: {path}")

    file_size = path.stat().st_size
    if not 0 < file_size <= MAX_IMAGE_BYTES:
        raise ImageProtocolError(
            f"image size must be between 1 and {MAX_IMAGE_BYTES} bytes"
        )

    extension = path.suffix.lower().lstrip(".")
    if extension not in IMAGE_EXTENSIONS:
        raise ImageProtocolError(
            "unsupported image extension; use jpg, jpeg, png, webp, bmp, gif, "
            "tif, or tiff"
        )

    transfer_id = transfer_id or secrets.token_hex(4)
    if TRANSFER_ID_PATTERN.fullmatch(transfer_id) is None:
        raise ImageProtocolError("transfer ID must be eight lowercase hex characters")

    total_chunks = (file_size + IMAGE_CHUNK_BYTES - 1) // IMAGE_CHUNK_BYTES
    return ImageMetadata(
        path=path,
        transfer_id=transfer_id,
        file_size=file_size,
        total_chunks=total_chunks,
        sha256=calculate_sha256(path),
        extension=extension,
    )


def build_start_packet(metadata: ImageMetadata) -> str:
    packet = (
        f"@I,S,{metadata.transfer_id},{metadata.file_size},"
        f"{metadata.total_chunks},{metadata.sha256},{metadata.extension}"
    )
    validate_payload(packet.encode("ascii"))
    return packet


def build_data_packet(transfer_id: str, chunk_index: int, chunk: bytes) -> str:
    if TRANSFER_ID_PATTERN.fullmatch(transfer_id) is None:
        raise ImageProtocolError("invalid transfer ID")
    if chunk_index < 0:
        raise ImageProtocolError("chunk index cannot be negative")
    if not 0 < len(chunk) <= IMAGE_CHUNK_BYTES:
        raise ImageProtocolError("invalid image chunk size")

    encoded = base64.b64encode(chunk).decode("ascii")
    packet = f"@I,D,{transfer_id},{chunk_index},{encoded}"
    validate_payload(packet.encode("ascii"))
    return packet


def build_end_packet(transfer_id: str) -> str:
    if TRANSFER_ID_PATTERN.fullmatch(transfer_id) is None:
        raise ImageProtocolError("invalid transfer ID")
    packet = f"@I,E,{transfer_id}"
    validate_payload(packet.encode("ascii"))
    return packet


def iter_image_data_packets(metadata: ImageMetadata) -> Iterator[tuple[int, str]]:
    bytes_read = 0
    with metadata.path.open("rb") as source:
        for chunk_index in range(metadata.total_chunks):
            chunk = source.read(IMAGE_CHUNK_BYTES)
            if not chunk:
                raise ImageProtocolError("image changed while it was being sent")
            bytes_read += len(chunk)
            yield chunk_index, build_data_packet(
                metadata.transfer_id, chunk_index, chunk
            )

        if bytes_read != metadata.file_size or source.read(1):
            raise ImageProtocolError("image changed while it was being sent")


class ImageReceiver:
    """Reassemble one inbound image transfer while tolerating duplicates."""

    def __init__(
        self,
        output_dir: Path,
        emit: Callable[[str], None] = print,
    ) -> None:
        self.output_dir = output_dir.expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.emit = emit
        self.active: IncomingImage | None = None
        self.completed_ids: deque[str] = deque(maxlen=32)
        self.reported_unknown_ids: set[str] = set()
        self.last_saved_path: Path | None = None

    def handle_packet(self, packet: bytes) -> bool:
        if not packet.startswith(b"@I,"):
            return False

        try:
            validate_payload(packet)
            message = packet.decode("ascii")
            fields = message.split(",")
            if len(fields) < 2:
                raise ImageProtocolError("missing image packet type")
            if fields[1] == "S":
                self._handle_start(fields)
            elif fields[1] == "D":
                self._handle_data(fields)
            elif fields[1] == "E":
                self._handle_end(fields)
            else:
                raise ImageProtocolError("unknown image packet type")
        except (UnicodeDecodeError, UwbNodeError, binascii.Error, OSError) as error:
            self.emit(f"[IMAGE ERROR] {error}")
        return True

    @staticmethod
    def _validate_transfer_id(transfer_id: str) -> None:
        if TRANSFER_ID_PATTERN.fullmatch(transfer_id) is None:
            raise ImageProtocolError("invalid transfer ID")

    def _handle_start(self, fields: list[str]) -> None:
        if len(fields) != 7:
            raise ImageProtocolError("invalid START packet")

        transfer_id, size_text, chunks_text, sha256, extension = fields[2:]
        self._validate_transfer_id(transfer_id)
        if SHA256_PATTERN.fullmatch(sha256) is None:
            raise ImageProtocolError("invalid SHA-256")
        if extension not in IMAGE_EXTENSIONS:
            raise ImageProtocolError("invalid image extension")

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
            self.emit(
                f"[IMAGE ABANDONED] id={self.active.transfer_id} "
                f"received={self.active.received_count}/{self.active.total_chunks}"
            )

        self.active = IncomingImage(
            transfer_id=transfer_id,
            file_size=file_size,
            total_chunks=total_chunks,
            sha256=sha256,
            extension=extension,
        )
        self.reported_unknown_ids.discard(transfer_id)
        self.emit(
            "\n[IMAGE START]\n"
            f"transfer_id={transfer_id}\n"
            f"size={file_size} bytes\n"
            f"chunks={total_chunks}\n"
            f"sha256={sha256}"
        )

    def _handle_data(self, fields: list[str]) -> None:
        if len(fields) != 5:
            raise ImageProtocolError("invalid DATA packet")

        transfer_id, index_text, encoded_chunk = fields[2:]
        self._validate_transfer_id(transfer_id)
        if transfer_id in self.completed_ids:
            return
        if self.active is None or self.active.transfer_id != transfer_id:
            if transfer_id not in self.reported_unknown_ids:
                self.emit(f"[IMAGE WARNING] DATA received before START: id={transfer_id}")
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
            self.emit(
                f"[IMAGE RECEIVE] {self.active.received_count}/"
                f"{self.active.total_chunks} ({progress}%)"
            )
            while self.active.next_progress <= progress:
                self.active.next_progress += 10

        if self.active.received_count == self.active.total_chunks:
            self._finalize_active()

    def _handle_end(self, fields: list[str]) -> None:
        if len(fields) != 3:
            raise ImageProtocolError("invalid END packet")
        transfer_id = fields[2]
        self._validate_transfer_id(transfer_id)
        if transfer_id in self.completed_ids:
            return
        if self.active is None or self.active.transfer_id != transfer_id:
            return
        if self.active.received_count == self.active.total_chunks:
            self._finalize_active()
            return

        missing_count = self.active.total_chunks - self.active.received_count
        first_missing = [
            str(index)
            for index, received in enumerate(self.active.received)
            if not received
        ][:10]
        self.emit(
            f"[IMAGE INCOMPLETE] missing={missing_count}, "
            f"first_missing={','.join(first_missing)}"
        )

    def _next_output_path(self, transfer: IncomingImage) -> Path:
        base_name = f"image_{transfer.transfer_id}"
        candidate = self.output_dir / f"{base_name}.{transfer.extension}"
        counter = 1
        while candidate.exists() or Path(f"{candidate}.part").exists():
            candidate = self.output_dir / f"{base_name}_{counter}.{transfer.extension}"
            counter += 1
        return candidate

    def _finalize_active(self) -> None:
        if self.active is None:
            return
        transfer = self.active
        actual_sha256 = hashlib.sha256(transfer.data).hexdigest()
        if actual_sha256 != transfer.sha256:
            self.emit(
                "[IMAGE ERROR] SHA-256 mismatch; image was not saved\n"
                f"expected={transfer.sha256}\nactual={actual_sha256}"
            )
            self.active = None
            return

        output_path = self._next_output_path(transfer)
        partial_path = Path(f"{output_path}.part")
        try:
            with partial_path.open("xb") as output_file:
                output_file.write(transfer.data)
                output_file.flush()
            partial_path.rename(output_path)
        except Exception:
            partial_path.unlink(missing_ok=True)
            raise

        self.last_saved_path = output_path
        self.emit(
            "\n[IMAGE SAVED]\n"
            f"path={output_path.resolve()}\n"
            f"size={transfer.file_size} bytes\n"
            f"duplicates={transfer.duplicate_count}\n"
            f"sha256={actual_sha256}"
        )
        self.completed_ids.append(transfer.transfer_id)
        self.active = None


class UwbNode:
    """Own the serial reader thread and serialize acknowledged transmissions."""

    def __init__(
        self,
        connection: serial.Serial,
        image_receiver: ImageReceiver,
        emit: Callable[[str], None] = print,
        ack_timeout: float = ACK_TIMEOUT_SECONDS,
    ) -> None:
        self.connection = connection
        self.image_receiver = image_receiver
        self.emit = emit
        self.ack_timeout = ack_timeout
        self._stop_event = threading.Event()
        self._feedback: queue.Queue[tuple[bool, str]] = queue.Queue()
        self._send_lock = threading.Lock()
        self._reader_thread: threading.Thread | None = None

    def start(self) -> None:
        if self._reader_thread is not None:
            return
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="uwb-serial-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)

    def _reader_loop(self) -> None:
        pending = bytearray()
        try:
            while not self._stop_event.is_set():
                chunk = self.connection.readline()
                if not chunk:
                    continue
                pending.extend(chunk)

                # A serial timeout may split one line. Dispatch only complete
                # newline-delimited messages from the firmware.
                while b"\n" in pending:
                    raw_line, _, remainder = pending.partition(b"\n")
                    pending = bytearray(remainder)
                    packet = raw_line.rstrip(b"\r")
                    if not packet:
                        continue
                    if packet == b"[UWB ACK]":
                        self._feedback.put((True, "[UWB ACK]"))
                    elif packet.startswith(b"[UWB ERROR]"):
                        self._feedback.put(
                            (False, packet.decode("utf-8", errors="replace"))
                        )
                    elif self.image_receiver.handle_packet(packet):
                        continue
                    elif packet.startswith(b"[UWB "):
                        self.emit(
                            f"[ESP32] {packet.decode('utf-8', errors='replace')}"
                        )
                    else:
                        text = packet.decode("utf-8", errors="replace")
                        self.emit(f"\n[RECEIVED]\n{text}")
        except (OSError, SerialException) as error:
            if not self._stop_event.is_set():
                message = f"serial reader stopped: {error}"
                self._feedback.put((False, message))
                self.emit(f"[ERROR] {message}")

    def send_line(self, message: str) -> str:
        payload = message.encode("utf-8")
        validate_payload(payload)

        with self._send_lock:
            while True:
                try:
                    self._feedback.get_nowait()
                except queue.Empty:
                    break

            self.connection.write(payload + b"\n")
            self.connection.flush()
            try:
                success, response = self._feedback.get(timeout=self.ack_timeout)
            except queue.Empty as error:
                raise UwbNodeError(
                    f"no [UWB ACK] from ESP32 within {self.ack_timeout:.1f} seconds"
                ) from error
            if not success:
                raise UwbNodeError(response)
            return response

    def send_text(self, message: str) -> None:
        if message.startswith("@I,") or message.startswith("[UWB "):
            raise UwbNodeError("text begins with a reserved protocol prefix")
        response = self.send_line(message)
        self.emit(f"{response}\n\n[SENT]\n{message}")

    def send_image(self, path: Path, packet_delay: float) -> ImageMetadata:
        metadata = prepare_image(path)
        self.emit(
            "\n[IMAGE SEND]\n"
            f"file={metadata.path}\n"
            f"size={metadata.file_size} bytes\n"
            f"chunks={metadata.total_chunks}\n"
            f"sha256={metadata.sha256}\n"
            f"transfer_id={metadata.transfer_id}"
        )

        self.send_line(build_start_packet(metadata))
        next_progress = 10
        for chunk_index, packet in iter_image_data_packets(metadata):
            self.send_line(packet)
            if packet_delay:
                time.sleep(packet_delay)
            progress = ((chunk_index + 1) * 100) // metadata.total_chunks
            if progress >= next_progress or chunk_index + 1 == metadata.total_chunks:
                self.emit(
                    f"[IMAGE SEND] {chunk_index + 1}/{metadata.total_chunks} "
                    f"({progress}%)"
                )
                while next_progress <= progress:
                    next_progress += 10

        self.send_line(build_end_packet(metadata.transfer_id))
        self.emit("[IMAGE SEND COMPLETE]")
        return metadata


def print_help(emit: Callable[[str], None] = print) -> None:
    emit(
        "Commands:\n"
        "  text MESSAGE   Send one UTF-8 text message (maximum 111 bytes)\n"
        "  image PATH     Send one image (maximum 10 MiB)\n"
        "  help           Show this help\n"
        "  quit           Exit"
    )


def run_interactive(
    node: UwbNode,
    image_delay: float,
    prompt: str = "uwb> ",
) -> None:
    print_help(node.emit)
    while True:
        try:
            command_line = input(prompt).strip()
        except EOFError:
            break
        if not command_line:
            continue

        command, separator, argument = command_line.partition(" ")
        command = command.lower()
        try:
            if command in {"quit", "exit"}:
                break
            if command == "help":
                print_help(node.emit)
            elif command == "text":
                if not separator or not argument:
                    raise UwbNodeError("usage: text MESSAGE")
                node.send_text(argument)
            elif command == "image":
                if not separator or not argument:
                    raise UwbNodeError("usage: image PATH")
                paths = shlex.split(argument)
                if len(paths) != 1:
                    raise UwbNodeError("usage: image PATH")
                node.send_image(Path(paths[0]), image_delay)
            else:
                raise UwbNodeError("unknown command; enter 'help'")
        except (OSError, UwbNodeError) as error:
            node.emit(f"[ERROR] {error}")


def run_self_check() -> None:
    max_chunks = (MAX_IMAGE_BYTES + IMAGE_CHUNK_BYTES - 1) // IMAGE_CHUNK_BYTES
    longest_start = build_start_packet(
        ImageMetadata(
            path=Path("unused.tiff"),
            transfer_id="ffffffff",
            file_size=MAX_IMAGE_BYTES,
            total_chunks=max_chunks,
            sha256="f" * 64,
            extension="tiff",
        )
    )
    longest_data = build_data_packet(
        "ffffffff", max_chunks - 1, bytes(IMAGE_CHUNK_BYTES)
    )
    for packet_type, packet in (("START", longest_start), ("DATA", longest_data)):
        if len(packet.encode("ascii")) > MAX_PAYLOAD_BYTES:
            raise AssertionError(
                f"maximum {packet_type} packet exceeds the UWB payload limit"
            )

    events: list[str] = []
    content = bytes(range(256)) + b"bidirectional-image-check"
    transfer_id = "1234abcd"
    sha256 = hashlib.sha256(content).hexdigest()
    total_chunks = (len(content) + IMAGE_CHUNK_BYTES - 1) // IMAGE_CHUNK_BYTES

    with tempfile.TemporaryDirectory(prefix="uwb-node-check-") as directory:
        receiver = ImageReceiver(Path(directory), events.append)
        start = (
            f"@I,S,{transfer_id},{len(content)},{total_chunks},{sha256},png"
        ).encode("ascii")
        receiver.handle_packet(start)
        packets = []
        for index in range(total_chunks):
            offset = index * IMAGE_CHUNK_BYTES
            chunk = content[offset : offset + IMAGE_CHUNK_BYTES]
            packets.append(build_data_packet(transfer_id, index, chunk).encode("ascii"))

        receiver.handle_packet(packets[0])
        receiver.handle_packet(packets[0])
        for packet in reversed(packets[1:]):
            receiver.handle_packet(packet)
        receiver.handle_packet(build_end_packet(transfer_id).encode("ascii"))

        if receiver.last_saved_path is None:
            raise AssertionError("image receiver did not save the completed transfer")
        if receiver.last_saved_path.read_bytes() != content:
            raise AssertionError("reconstructed image differs from its source")

    print(
        "[SELF CHECK PASS]\n"
        f"chunk_bytes={IMAGE_CHUNK_BYTES}\n"
        f"longest_start_packet={len(longest_start.encode('ascii'))} bytes\n"
        f"longest_data_packet={len(longest_data.encode('ascii'))} bytes\n"
        f"payload_limit={MAX_PAYLOAD_BYTES} bytes"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send and receive text or images through a bidirectional UWB node."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="ESP32 serial port")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="serial baud rate")
    parser.add_argument(
        "--name",
        choices=("laptop", "jetson"),
        help="optional local name used in the interactive prompt",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"received image directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--image-delay",
        type=float,
        default=DEFAULT_IMAGE_DELAY_SECONDS,
        help="seconds to wait after each image DATA packet (default: 0.002)",
    )
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--text", help="send one text message and exit")
    action.add_argument("--image", type=Path, help="send one image and exit")
    action.add_argument(
        "--self-test",
        action="store_true",
        help="run protocol and image reconstruction checks without opening serial",
    )
    args = parser.parse_args()
    if args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if args.image_delay < 0:
        parser.error("--image-delay must be zero or greater")
    return args


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_check()
        return 0

    console_lock = threading.Lock()

    def emit(message: str) -> None:
        with console_lock:
            print(message, flush=True)

    try:
        image_receiver = ImageReceiver(args.output_dir, emit)
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.1,
            write_timeout=1.0,
        ) as connection:
            emit(f"[INFO] Opening {args.port} at {args.baud} baud...")
            # Clear only bytes left from before this session. Do not clear
            # again after the startup wait: the peer may already have sent a
            # frame and received its radio ACK during that interval.
            connection.reset_input_buffer()
            node = UwbNode(connection, image_receiver, emit)
            node.start()
            try:
                # Read boot/status lines immediately, but wait before allowing
                # local sends so the ESP32 and DW1000 can finish setup.
                time.sleep(STARTUP_DELAY_SECONDS)
                emit(f"[INFO] Connected to {args.port} at {args.baud} baud")
                if args.text is not None:
                    node.send_text(args.text)
                elif args.image is not None:
                    node.send_image(args.image, args.image_delay)
                else:
                    emit(f"[INFO] Receiving images into {args.output_dir.expanduser()}")
                    prompt = f"{args.name}> " if args.name else "uwb> "
                    run_interactive(node, args.image_delay, prompt)
            finally:
                node.close()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
        return 0
    except (OSError, SerialException, UwbNodeError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
