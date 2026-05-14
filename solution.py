from dataclasses import dataclass
from math import sqrt

G = 9.8
SMOKE_RADIUS = 10.0
SMOKE_DURATION = 20.0

TARGET = (0.0, 2000.0, 0.0)
M1_0 = (20000.0, 0.0, 2000.0)
FY1_0 = (17000.0, 0.0, 1800.0)
MISSILE_SPEED = 300.0
DRONE_SPEED = 120.0


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_mul(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def norm(a):
    return sqrt(dot(a, a))


def normalize(v):
    n = norm(v)
    return (v[0]/n, v[1]/n, v[2]/n)


C1 = normalize(v_sub(TARGET, M1_0))


def missile_pos(t):
    return v_add(M1_0, v_mul(C1, MISSILE_SPEED * t))


def missile_hit_time():
    return norm(v_sub(TARGET, M1_0)) / MISSILE_SPEED


def drone_pos(t):
    return v_add(FY1_0, v_mul(C1, DRONE_SPEED * t))


@dataclass
class SmokeBomb:
    t_release: float

    def burst_state(self):
        return drone_pos(self.t_release), v_mul(C1, DRONE_SPEED)

    def center(self, t):
        dt = t - self.t_release
        p0, v0 = self.burst_state()
        ballistic = (0.0, 0.0, -0.5 * G * dt * dt)
        return v_add(v_add(p0, v_mul(v0, dt)), ballistic)

    def active(self, t):
        return self.t_release <= t <= self.t_release + SMOKE_DURATION


def point_segment_distance(p, a, b):
    ab = v_sub(b, a)
    ap = v_sub(p, a)
    ab2 = dot(ab, ab)
    if ab2 == 0:
        return norm(v_sub(p, a))
    t = dot(ap, ab) / ab2
    t = max(0.0, min(1.0, t))
    proj = v_add(a, v_mul(ab, t))
    return norm(v_sub(p, proj))


def blocked_at(t, bombs):
    m = missile_pos(t)
    for bomb in bombs:
        if bomb.active(t):
            c = bomb.center(t)
            if point_segment_distance(c, TARGET, m) <= SMOKE_RADIUS:
                return True
    return False


def effective_duration(bombs, dt=0.01):
    t_end = min(missile_hit_time(), max(b.t_release + SMOKE_DURATION for b in bombs))
    t = 0.0
    blocked = 0
    while t <= t_end:
        if blocked_at(t, bombs):
            blocked += 1
        t += dt
    return blocked * dt


if __name__ == '__main__':
    bombs = [SmokeBomb(0.0), SmokeBomb(3.6)]
    duration = effective_duration(bombs)
    print('C1 =', C1)
    print(f'M1命中目标时间: {missile_hit_time():.3f} s')
    print(f'有效遮蔽时长(近似): {duration:.3f} s')
