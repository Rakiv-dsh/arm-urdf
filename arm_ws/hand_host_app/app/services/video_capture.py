"""Camera helpers with a V4L2 MJPEG fallback for WSL/USB cameras."""

from __future__ import annotations

import mmap
import os
import select
import struct
from dataclasses import dataclass

import numpy as np

from app.platform_compat import is_wsl

try:
    import fcntl
except ImportError:  # Windows does not provide fcntl.
    fcntl = None


_VIDIOC_QUERYCAP = 0x80685600
_VIDIOC_ENUM_FMT = 0xC0405602
_VIDIOC_S_FMT = 0xC0D05605
_VIDIOC_REQBUFS = 0xC0145608
_VIDIOC_QUERYBUF = 0xC0585609
_VIDIOC_QBUF = 0xC058560F
_VIDIOC_DQBUF = 0xC0585611
_VIDIOC_STREAMON = 0x40045612
_VIDIOC_STREAMOFF = 0x40045613

_V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
_V4L2_BUF_TYPE_META_CAPTURE = 13
_V4L2_MEMORY_MMAP = 1
_V4L2_FIELD_NONE = 1
_V4L2_CAP_VIDEO_CAPTURE = 0x00000001
_V4L2_PIX_FMT_MJPEG = int.from_bytes(b"MJPG", "little")


def _require_fcntl() -> None:
    if fcntl is None:
        raise RuntimeError("V4L2 helpers require a POSIX environment with fcntl support")


@dataclass(frozen=True)
class CameraDeviceInfo:
    path: str
    exists: bool
    is_capture: bool
    formats: tuple[str, ...]
    message: str = ""


def camera_path(camera_id: int | str) -> str:
    return f"/dev/video{int(camera_id)}"


def parse_camera_source(source: int | str) -> int | str:
    if isinstance(source, int):
        return source
    text = str(source).strip()
    try:
        return int(text)
    except ValueError:
        return text


def inspect_v4l2_device(path: str) -> CameraDeviceInfo:
    _require_fcntl()
    if not os.path.exists(path):
        return CameraDeviceInfo(path, False, False, (), f"未发现 {path}")

    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError as exc:
        return CameraDeviceInfo(path, True, False, (), f"无法打开 {path}: {exc}")

    try:
        cap = bytearray(104)
        fcntl.ioctl(fd, _VIDIOC_QUERYCAP, cap, True)
        caps = struct.unpack_from("I", cap, 80)[0]
        device_caps = struct.unpack_from("I", cap, 88)[0]
        effective_caps = device_caps or caps
        is_capture = bool(effective_caps & _V4L2_CAP_VIDEO_CAPTURE)
        formats = _enum_formats(fd, _V4L2_BUF_TYPE_VIDEO_CAPTURE if is_capture else _V4L2_BUF_TYPE_META_CAPTURE)
        if not is_capture:
            return CameraDeviceInfo(path, True, False, formats, f"{path} 不是视频采集节点")
        return CameraDeviceInfo(path, True, True, formats)
    except OSError as exc:
        return CameraDeviceInfo(path, True, False, (), f"{path} V4L2 查询失败: {exc}")
    finally:
        os.close(fd)


def _enum_formats(fd: int, buffer_type: int) -> tuple[str, ...]:
    formats: list[str] = []
    index = 0
    while True:
        desc = bytearray(64)
        struct.pack_into("II", desc, 0, index, buffer_type)
        try:
            fcntl.ioctl(fd, _VIDIOC_ENUM_FMT, desc, True)
        except OSError:
            break
        pixelformat = struct.unpack_from("I", desc, 44)[0]
        formats.append(pixelformat.to_bytes(4, "little").decode(errors="replace"))
        index += 1
    return tuple(formats)


class V4L2MjpegCapture:
    """Small V4L2 mmap MJPEG reader compatible with cv2.VideoCapture.read()."""

    def __init__(self, path: str, width: int = 640, height: int = 480):
        import cv2

        self.path = path
        self.width = int(width)
        self.height = int(height)
        self._cv2 = cv2
        self._fd: int | None = None
        self._buffers: list[mmap.mmap] = []
        self._opened = False
        self._open()

    def isOpened(self) -> bool:
        return self._opened

    def read(self):
        if not self._opened or self._fd is None:
            return False, None

        for _attempt in range(8):
            ready, _, _ = select.select([self._fd], [], [], 1.0)
            if not ready:
                return False, None

            buf = bytearray(88)
            struct.pack_into("II", buf, 0, 0, _V4L2_BUF_TYPE_VIDEO_CAPTURE)
            struct.pack_into("I", buf, 60, _V4L2_MEMORY_MMAP)
            try:
                fcntl.ioctl(self._fd, _VIDIOC_DQBUF, buf, True)
            except BlockingIOError:
                continue

            index = struct.unpack_from("I", buf, 0)[0]
            bytes_used = struct.unpack_from("I", buf, 8)[0]
            try:
                payload = bytes(self._buffers[index][:bytes_used])
                frame = self._decode_mjpeg(payload)
            finally:
                fcntl.ioctl(self._fd, _VIDIOC_QBUF, buf, True)

            if frame is not None:
                return True, frame

        return False, None

    def release(self) -> None:
        if self._fd is not None:
            capture_type = struct.pack("I", _V4L2_BUF_TYPE_VIDEO_CAPTURE)
            try:
                fcntl.ioctl(self._fd, _VIDIOC_STREAMOFF, capture_type)
            except OSError:
                pass
        for mapped in self._buffers:
            try:
                mapped.close()
            except OSError:
                pass
        self._buffers.clear()
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._opened = False

    def _open(self) -> None:
        _require_fcntl()
        info = inspect_v4l2_device(self.path)
        if not info.is_capture:
            raise RuntimeError(info.message or f"{self.path} 不是视频采集节点")
        if "MJPG" not in info.formats:
            raise RuntimeError(f"{self.path} 不支持 MJPG，当前格式: {', '.join(info.formats) or '未知'}")

        self._fd = os.open(self.path, os.O_RDWR | os.O_NONBLOCK)
        try:
            self._set_format()
            self._request_buffers()
            capture_type = struct.pack("I", _V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self._fd, _VIDIOC_STREAMON, capture_type)
            self._opened = True
        except Exception:
            self.release()
            raise

    def _set_format(self) -> None:
        assert self._fd is not None
        fmt = bytearray(208)
        struct.pack_into("I", fmt, 0, _V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into(
            "IIIIII",
            fmt,
            8,
            self.width,
            self.height,
            _V4L2_PIX_FMT_MJPEG,
            _V4L2_FIELD_NONE,
            0,
            0,
        )
        fcntl.ioctl(self._fd, _VIDIOC_S_FMT, fmt, True)
        self.width = struct.unpack_from("I", fmt, 8)[0]
        self.height = struct.unpack_from("I", fmt, 12)[0]

    def _request_buffers(self) -> None:
        assert self._fd is not None
        request = bytearray(20)
        struct.pack_into("III", request, 0, 4, _V4L2_BUF_TYPE_VIDEO_CAPTURE, _V4L2_MEMORY_MMAP)
        fcntl.ioctl(self._fd, _VIDIOC_REQBUFS, request, True)
        count = struct.unpack_from("I", request, 0)[0]
        if count <= 0:
            raise RuntimeError(f"{self.path} 未分配 V4L2 缓冲区")

        for index in range(count):
            buf = bytearray(88)
            struct.pack_into("II", buf, 0, index, _V4L2_BUF_TYPE_VIDEO_CAPTURE)
            struct.pack_into("I", buf, 60, _V4L2_MEMORY_MMAP)
            fcntl.ioctl(self._fd, _VIDIOC_QUERYBUF, buf, True)
            length = struct.unpack_from("I", buf, 72)[0]
            offset = struct.unpack_from("I", buf, 64)[0]
            mapped = mmap.mmap(
                self._fd,
                length,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE,
                offset=offset,
            )
            self._buffers.append(mapped)
            fcntl.ioctl(self._fd, _VIDIOC_QBUF, buf, True)

    def _decode_mjpeg(self, payload: bytes):
        start = payload.find(b"\xff\xd8")
        end = payload.rfind(b"\xff\xd9")
        if start >= 0 and end > start:
            payload = payload[start : end + 2]
        else:
            return None

        image = np.frombuffer(payload, dtype=np.uint8)
        return self._quiet_imdecode(image)

    def _quiet_imdecode(self, image: np.ndarray):
        old_stderr = os.dup(2)
        try:
            with open(os.devnull, "wb") as devnull:
                os.dup2(devnull.fileno(), 2)
                return self._cv2.imdecode(image, self._cv2.IMREAD_COLOR)
        finally:
            os.dup2(old_stderr, 2)
            os.close(old_stderr)


def open_camera_capture(camera_id: int | str, width: int = 640, height: int = 480):
    import cv2

    source = parse_camera_source(camera_id)
    if not isinstance(source, int):
        capture = cv2.VideoCapture(source)
        if capture.isOpened():
            return capture
        raise RuntimeError(f"无法打开摄像头/视频源: {source}")

    if os.name == "posix":
        path = camera_path(source)
        info = inspect_v4l2_device(path)
        if not info.is_capture:
            raise RuntimeError(info.message or f"{path} 不是可用摄像头")
        if is_wsl() and "MJPG" in info.formats:
            return V4L2MjpegCapture(path, width=width, height=height)

    capture = cv2.VideoCapture(source)
    if capture.isOpened():
        return capture

    if os.name == "posix":
        path = camera_path(source)
        info = inspect_v4l2_device(path)
        if "MJPG" in info.formats:
            return V4L2MjpegCapture(path, width=width, height=height)
        detail = f"；格式: {', '.join(info.formats) or '未知'}" if info.exists else ""
        raise RuntimeError((info.message or f"无法打开摄像头 {source}") + detail)

    raise RuntimeError(f"无法打开摄像头 {source}")
