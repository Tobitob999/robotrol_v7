import time
from dataclasses import dataclass


@dataclass
class Frame:
    image: object
    timestamp: float


class CameraInterface:
    def get_frame(self) -> Frame:
        raise NotImplementedError


class MockCamera(CameraInterface):
    color_order = "rgb"

    def get_frame(self) -> Frame:
        return Frame(image=None, timestamp=time.time())


class ImageFileCamera(CameraInterface):
    def __init__(self, path: str):
        self.path = path
        self._cv2 = None
        self.color_order = "bgr"

    def _load_cv2(self):
        if self._cv2 is None:
            import cv2
            self._cv2 = cv2

    def get_frame(self) -> Frame:
        self._load_cv2()
        image = self._cv2.imread(self.path, self._cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("Failed to load image file")
        return Frame(image=image, timestamp=time.time())


class OpenCVCamera(CameraInterface):
    def __init__(self, device_index: int, width: int | None = None, height: int | None = None, fps: int | None = None, backend: str | None = None):
        import cv2
        self.cv2 = cv2
        backend_map = {
            "dshow": self.cv2.CAP_DSHOW,
            "msmf": self.cv2.CAP_MSMF,
            "any": self.cv2.CAP_ANY,
        }
        api_pref = backend_map.get((backend or "any").lower(), self.cv2.CAP_ANY)
        self.cap = self.cv2.VideoCapture(device_index, api_pref)
        self.color_order = "bgr"
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera device")
        if width is not None and int(width) > 0:
            self.cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height is not None and int(height) > 0:
            self.cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps is not None and int(fps) > 0:
            self.cap.set(self.cv2.CAP_PROP_FPS, int(fps))

    def get_frame(self) -> Frame:
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("Failed to read camera frame")
        return Frame(image=frame, timestamp=time.time())


def build_camera(config: dict) -> CameraInterface:
    source = (config.get("source") or "mock").lower()
    if source == "mock":
        return MockCamera()
    if source == "file":
        return ImageFileCamera(config.get("image_path", ""))
    if source == "device":
        return OpenCVCamera(
            int(config.get("device_index", 0)),
            width=config.get("width"),
            height=config.get("height"),
            fps=config.get("fps"),
            backend=config.get("backend"),
        )
    raise ValueError("Unknown camera source")
