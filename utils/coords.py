import numpy as np


def to_homogeneous(pts):
    shape = pts.shape
    shape[-1] += 1

    out_pts = np.ones(shape)
    out_pts[..., :-1] = pts

    return out_pts

def normalize_homogeneous(pts):
    out_pts = pts / pts[..., -1].reshape((*pts.shape[:-1], 1))
    return out_pts

def from_homogeneous(pts):
    return normalize_homogeneous(pts)[..., :-1]
