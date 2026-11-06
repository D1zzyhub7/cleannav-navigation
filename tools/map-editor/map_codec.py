"""Lossless {-1,0,100} OccupancyGrid / Nav2 trinary PGM conversion."""
import numpy as np


def decode_trinary(image, *, free_thresh=.25, occupied_thresh=.65):
    if not 0 < free_thresh < occupied_thresh < 1:
        raise ValueError('trinary thresholds must satisfy 0 < free < occupied < 1')
    pixels = np.asarray(image)
    if pixels.ndim != 2 or np.any(pixels < 0) or np.any(pixels > 255):
        raise ValueError('trinary image must be a grayscale image')
    probability = (255.0 - pixels.astype(float)) / 255.0
    result = np.full(pixels.shape, -1, dtype=np.int16)
    result[probability > occupied_thresh] = 100
    result[probability < free_thresh] = 0
    return np.flipud(result)


def encode_trinary(grid, *, free_thresh=.25, occupied_thresh=.65):
    values = np.asarray(grid)
    if values.ndim != 2 or np.any(~np.isin(values, [-1, 0, 100])):
        raise ValueError('lossless trinary export requires a {-1,0,100} grid')
    # 205 is free at free_thresh=.25. Choose an actual unknown gray level;
    # verify using exactly the loader's probability/strict-boundary rules.
    unknown = 127
    if not free_thresh <= (255 - unknown) / 255 <= occupied_thresh:
        unknown = round(255 * (1 - (free_thresh + occupied_thresh) / 2))
    free = 254 if 1 / 255 < free_thresh else 255
    image = np.flipud(np.where(values == -1, unknown, np.where(values == 100, 0, free))).astype('uint8')
    if not np.array_equal(decode_trinary(image, free_thresh=free_thresh,
                                       occupied_thresh=occupied_thresh), values):
        raise ValueError('PGM cannot represent this trinary grid at the supplied thresholds')
    return image
