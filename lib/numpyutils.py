import numpy as np

def vector_unit(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def vectors_angle(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    print(v1)
    print(v2)
    v1_u = vector_unit(v1)
    v2_u = vector_unit(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def vector_intersect(a1, a2, b1, b2):
    """ 
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return (float('inf'), float('inf'))
    return (x/z, y/z)


def rotation_matrix_2d(deg):
    rad = np.deg2rad(deg)
    c, s = np.cos(rad), np.sin(rad)
    R = np.array(((c, -s), (s, c)))
    return R

def rotation_matrix_2d_x(deg):
    rad = np.deg2rad(deg)
    c, s = np.cos(rad), np.sin(rad)
    R = np.array((
        (s, c),
        (c, -s)
        ))
    return R