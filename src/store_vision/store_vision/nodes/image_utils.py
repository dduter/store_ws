import numpy as np
from sensor_msgs.msg import Image


_ENCODINGS = {
    'bgr8': (np.uint8, 3),
    'rgb8': (np.uint8, 3),
    '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1),
}


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    """행 패딩과 바이트 순서를 반영하고 깊이는 mm 단위로 반환한다."""
    dtype, channels = _ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    pixel_step = channels * dtype.itemsize
    if msg.step < msg.width * pixel_step or len(msg.data) < msg.height * msg.step:
        raise ValueError('이미지의 행 크기 또는 데이터 길이가 올바르지 않습니다.')
    shape = (msg.height, msg.width)
    strides = (msg.step, pixel_step)
    if channels > 1:
        shape += (channels,)
        strides += (dtype.itemsize,)
    array = np.ndarray(shape, dtype=dtype, buffer=msg.data, strides=strides)
    if msg.encoding == '32FC1':
        return array.astype(np.float32) * 1000.0
    return array.astype(dtype.newbyteorder('='), copy=False)


def numpy_to_image_msg(arr: np.ndarray, encoding: str, stamp, frame_id: str) -> Image:
    """배열을 ROS 이미지로 직렬화한다. 32FC1 배열은 미터 단위로 받는다."""
    dtype, channels = _ENCODINGS[encoding]
    pixels = np.ascontiguousarray(arr, dtype=np.dtype(dtype).newbyteorder('<'))
    if channels == 1 and pixels.ndim == 3 and pixels.shape[2] == 1:
        pixels = pixels[:, :, 0]
    if (channels == 1 and pixels.ndim != 2) or (
        channels > 1 and (pixels.ndim != 3 or pixels.shape[2] != channels)
    ):
        raise ValueError('배열의 모양과 이미지 인코딩이 일치하지 않습니다.')
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = pixels.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * channels * pixels.itemsize
    msg.data = pixels.tobytes()
    return msg
