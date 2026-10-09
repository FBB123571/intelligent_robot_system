"""Small ROS message adapters; geometry stays in independently tested modules."""
from .depth import finite
from .motion import Grid, yaw


def same_frame(left, right):
    return bool(left) and left.lstrip('/') == right.lstrip('/')


def grid_message(msg, frame):
    if not same_frame(msg.header.frame_id, frame):
        raise ValueError('costmap_frame_mismatch')
    p, q = msg.info.origin.position, msg.info.origin.orientation
    return Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, (p.x, p.y, yaw(q)))


def transform_pose(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    if not all(finite(v) for v in values) or abs(sum(v*v for v in values[3:]) - 1.) > .1:
        raise ValueError('invalid_base_transform')
    return t.x, t.y, yaw(q)
