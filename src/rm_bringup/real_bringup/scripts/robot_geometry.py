"""Physical body geometry shared by self-return filtering and close-wall navigation."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class RobotGeometry:
    length: float = .633
    width: float = .526
    height: float = .42
    padding: float = .02
    spin_radius: float = .47

    def __post_init__(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError('Robot dimensions must be finite')
        if min(self.length, self.width, self.height) <= 0 or self.padding < 0:
            raise ValueError('Robot dimensions must be positive and padding nonnegative')
        if self.spin_radius < math.hypot(self.length/2+self.padding, self.width/2+self.padding):
            raise ValueError('Spin radius must cover the padded body corners')

    def outside_body(self, points):
        """Remove only the physical body volume, never its padded safety envelope."""
        epsilon = 1e-6
        inside = ((np.abs(points[:, 0]) <= self.length/2+epsilon) &
                  (np.abs(points[:, 1]) <= self.width/2+epsilon) &
                  (points[:, 2] >= -epsilon) & (points[:, 2] <= self.height+epsilon))
        return points[~inside]

    def footprint(self, yaw, spinning=False):
        """Footprint in nav_base_link, whose heading is fixed to odom."""
        if spinning:
            # Circumscribe the swept disk rather than cutting its edges off.
            angles = np.arange(32)*2*math.pi/32
            radius = self.spin_radius/math.cos(math.pi/32)
            return np.column_stack((radius*np.cos(angles), radius*np.sin(angles)))
        x, y = self.length/2+self.padding, self.width/2+self.padding
        points = np.array([[-x, -y], [x, -y], [x, y], [-x, y]])
        c, s = math.cos(yaw), math.sin(yaw)
        return points @ np.array([[c, s], [-s, c]])

    def spin_clear(self, points, scan_age):
        return (-.05 <= scan_age < .5 and bool(points) and
                all(math.isfinite(x) and math.isfinite(y) and
                    math.hypot(x, y) >= self.spin_radius for x, y in points))

    def guarded_velocity(self, vx, vy, points):
        """Braking limit for a translating rectangle, including its corners."""
        speed=math.hypot(vx,vy)
        if speed < 1e-9:
            return vx,vy
        direction=(vx/speed,vy/speed)
        half=(self.length/2+self.padding,self.width/2+self.padding)
        clearance=math.inf
        for point in points:
            entry,exit_distance=-math.inf,math.inf
            for p,u,h in zip(point,direction,half):
                if abs(u)<1e-9:
                    if abs(p)>h:
                        exit_distance=-math.inf;break
                else:
                    a,b=(p-h)/u,(p+h)/u
                    entry=max(entry,min(a,b));exit_distance=min(exit_distance,max(a,b))
            if exit_distance>=max(0.,entry):
                clearance=min(clearance,max(0.,entry))
        allowed=min(speed,clearance/.4,math.sqrt(2.*clearance))
        return vx*allowed/speed,vy*allowed/speed
