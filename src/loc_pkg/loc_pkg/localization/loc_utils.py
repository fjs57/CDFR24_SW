import numpy as np

def cicle_intersections(x1:float, y1:float, r1:float, x2:float, y2:float, r2:float):
    d = np.sqrt((x2-x1)**2+(y2-y1)**2)
    
    if d > (r1+r2) : return None
    if d < 0.0001 : return None

    l = (r1**2-r2**2+d**2)/(2*d)
    h = np.sqrt(r1**2-l**2)
    x = x1 + l/d*(x2-x1)
    dx = h/d*(y2-y1)
    y = y1 + l/d*(y2-y1)
    dy = h/d*(x2-x1)
    ax, ay = x+dx, y-dy
    bx, by = x-dx, y+dy


    return [[ax, ay],[bx, by]]

def get_angle_two_points(x1:float, y1:float, x2:float, y2:float):
    return np.arctan2(y2-y1, x2-x1)%(2*np.pi)

def get_angle_three_points(ax:float, ay:float, ox:float, oy:float, bx:float, by:float):
    A = get_angle_two_points(ox, oy, ax, ay)
    B = get_angle_two_points(ox, oy, bx, by)
    return (A-B)%(2*np.pi)