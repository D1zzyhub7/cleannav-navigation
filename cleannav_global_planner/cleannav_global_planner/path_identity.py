"""Planner/Executor共用的路径几何身份，不包含消息时间或Action身份。"""
import hashlib
import math
import struct


def path_signature(path):
    """完整有序XYZ/四元数与有效frame，使用无舍入的网络序float64编码。

    Pose空frame继承Path frame；混合frame、非有限几何、零四元数拒绝。
    四元数q/-q及正负零视为等价；不归一化或量化其他输入。
    此摘要不是Action UUID、目标generation或导航完成证明。
    """
    if not path.poses:
        return ''
    frame = path.header.frame_id
    if not frame or len(path.poses) < 2:
        raise ValueError('path requires a frame and at least two poses')
    encoded_frame = frame.encode('utf-8')
    digest = hashlib.sha256(b'CLEANNAV_PATH_GEOMETRY_V2\0')
    digest.update(struct.pack('!I', len(encoded_frame)))
    digest.update(encoded_frame)
    digest.update(struct.pack('!Q', len(path.poses)))
    for index, stamped in enumerate(path.poses):
        if stamped.header.frame_id and stamped.header.frame_id != frame:
            raise ValueError(f'pose[{index}] frame mismatch')
        position, orientation = stamped.pose.position, stamped.pose.orientation
        xyz = (position.x, position.y, position.z)
        quat = (orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(value) for value in (*xyz, *quat)):
            raise ValueError(f'pose[{index}] geometry non-finite')
        if not any(quat):
            raise ValueError(f'pose[{index}] zero quaternion')
        # w,z,y,x中第一个非零分量决定符号，不改变实际朝向。
        if next(value for value in reversed(quat) if value != 0.) < 0.:
            quat = tuple(-value for value in quat)
        values = tuple(0. if value == 0. else value for value in (*xyz, *quat))
        digest.update(struct.pack('!7d', *values))
    return digest.hexdigest()
