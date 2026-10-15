"""Offline replica of Flight.cs control loops, used for the 2026-10-13 audit.

Reproduces, one axis at a time, what Flight.FixedUpdate computes and what PhysX then
integrates (semi-implicit Euler at Time.fixedDeltaTime = 0.02 s):
  - roll/pitch: PD on angle (Kp_rp, Kd_rp), mixer + saturation (Flight.cs:736-775),
    angular drag (:896), Rigidbody.angularDamping, the 3 rad/s clamp (:559-568)
  - yaw: P on rate (Kp_yaw)
  - altitude hand-off after the kinematic launch (Kp_alt, Kd_alt, dragY)

Run:  python tools/flight-audit/flight_loop_replica.py
It does not touch Unity. Constants are copied from Flight.cs; keep them in sync if
Flight.cs changes.
"""
import math

# Flight.cs constants
kf = 9.13e-6
w_max = 1256.6
F_max = kf * w_max ** 2
Kp_rp, Kd_rp = 10.0, 0.06
Kp_yaw = 0.55
ang_drag = 0.005          # angularDragCoeff
ang_damp = 0.02           # Rigidbody.angularDamping set in Start()
clamp = 3.0               # maxAngularVelocity
dt = 0.02                 # ProjectSettings/TimeManager.asset Fixed Timestep
g = 9.81
roll_sign = [-1, 1, 1, -1]  # motorRollSign, order FR, FL, BL, BR


def mix(total_thrust, tau, arm_eff):
    """Flight.cs:736-775 for one axis. Returns delivered thrust, delivered torque, saturated?"""
    F = [total_thrust / 4 + s * tau / (4 * arm_eff) for s in roll_sign]
    lo, hi = min(F), max(F)
    if lo < 0:
        F = [f - lo for f in F]
        hi -= lo
    if hi > F_max:
        F = [f - (hi - F_max) for f in F]
    F = [min(max(f, 0.0), F_max) for f in F]
    sat = min(F) <= 1e-9 or max(F) >= F_max - 1e-6
    return sum(F), sum(s * f * arm_eff for s, f in zip(roll_sign, F)), sat


def run_attitude(I, m, arm_eff, th0=0.0, w0=0.0, th_cmd=0.0, T=3.0):
    th, w = th0, w0
    hist, extra, sat_steps = [], 0.0, 0
    for _ in range(int(T / dt)):
        if abs(w) > clamp:
            w = math.copysign(clamp, w)
        hover = m * g
        thrust = hover * min(1 / max(math.cos(th), 1e-3), 1.15)
        tau_cmd = Kp_rp * (th_cmd - th) - Kd_rp * w
        delivered, tau, sat = mix(thrust, tau_cmd, arm_eff)
        sat_steps += sat
        extra = max(extra, delivered * math.cos(th) / hover - 1)
        tau -= ang_drag * w
        w += dt * tau / I
        w /= 1 + dt * ang_damp
        w = max(-6.0, min(6.0, w))  # Rigidbody.maxAngularVelocity = 2 * 3
        th += dt * w
        hist.append(math.degrees(th))
    return hist, extra, sat_steps


def settle_time(hist, target_deg, tol_deg):
    last = 0
    for i, d in enumerate(hist):
        if abs(d - target_deg) > tol_deg:
            last = i + 1
    return last * dt


def attitude_table():
    print("ROLL/PITCH loop (fixed gains Kp=%.1f, Kd=%.2f, dt=%.2f s)" % (Kp_rp, Kd_rp, dt))
    print("  analytic small-signal stability limit: I > %.5f kg m^2"
          % ((Kp_rp * dt * dt + 2 * (Kd_rp + ang_drag) * dt) / 4))
    print("  %-9s %-5s %-6s %-6s | %-28s | %s" % ("I", "m", "armEff", "zeta", "kick w0=2 rad/s: peak, settle",
                                                   "17.5 deg step: overshoot, settle, max extra thrust"))
    cases = [(0.0008, 1.7, 0.15, ""), (0.0012, 1.7, 0.15, ""), (0.0016, 1.7, 0.15, ""),
             (0.002, 1.7, 0.15, ""), (0.003, 2.0, 0.16 / math.sqrt(2), "stock"),
             (0.006167, 1.7, 0.15, "cube quad"), (0.012, 1.7, 0.15, ""),
             (0.03, 3.0, 0.25, ""), (0.1, 5.0, 0.40, "")]
    for I, m, a, label in cases:
        zeta = (Kd_rp + ang_drag) / (2 * math.sqrt(Kp_rp * I))
        h, _, _ = run_attitude(I, m, a, w0=2.0)
        peak = max(abs(x) for x in h)
        st = settle_time(h, 0.0, 0.5)
        step = 17.5
        h2, extra, _ = run_attitude(I, m, a, th_cmd=math.radians(step))
        ov = (max(h2) - step) / step * 100
        st2 = settle_time(h2, step, 0.05 * step)
        nst = ">3 s (no settle)"
        print("  %-9g %-5.2f %-6.3f %-6.3f | %5.2f deg, %-16s | %5.1f %%, %-16s %5.1f %%  %s"
              % (I, m, a, zeta, peak, nst if st >= 3.0 else "%.2f s" % st,
                 ov, nst if st2 >= 3.0 else "%.2f s" % st2, extra * 100, label))
    print("  tiny kick (0.01 rad/s), last 1 s of 6 s:")
    for I in (0.0012, 0.0016, 0.0017, 0.002):
        h, _, _ = run_attitude(I, 1.7, 0.15, w0=0.01, T=6.0)
        print("    I=%.4f  max |angle| = %.3f deg   last samples %s"
              % (I, max(abs(x) for x in h[-50:]), ["%.2f" % x for x in h[-4:]]))


def yaw_table():
    print("\nYAW rate loop (P only, Kp_yaw=%.2f)" % Kp_yaw)
    print("  analytic stability limit: Iyy > %.5f kg m^2" % ((Kp_yaw + ang_drag) * dt / 2))
    for I in (0.003, 0.005, 0.0055, 0.006, 0.0075, 0.010667, 0.03):
        w, seq = 0.5, []
        for _ in range(150):
            tau = -Kp_yaw * w - ang_drag * w
            w += dt * tau / I
            w /= 1 + dt * ang_damp
            w = max(-clamp, min(clamp, w))
            seq.append(w)
        label = " (stock)" if I == 0.006 else (" (cube quad)" if I == 0.010667 else "")
        print("  Iyy=%.4f  per-step factor %+.3f  |yaw rate| after 1 s %.4f, after 3 s %.4f rad/s%s"
              % (I, 1 - dt * (Kp_yaw + ang_drag) / I, abs(seq[49]), abs(seq[-1]), label))


def limits():
    print("\nTHRUST limits: F_max/motor = %.2f N, 4 motors = %.2f N" % (F_max, 4 * F_max))
    print("  cannot hover above %.2f kg" % (4 * F_max / g))
    print("  full climb command (1 + 1.8) x hover saturates above %.2f kg" % (4 * F_max / (2.8 * g)))
    for m in (2.0, 1.7):
        print("  hover motor speed for %.1f kg: %.1f rad/s" % (m, math.sqrt(m * g / 4 / kf)))
    m, x, v, peak = 1.7, 0.0, 0.5, 0.0
    for _ in range(500):
        a = g * (-0.5 * x - 0.8 * v) - 0.12 * v / m
        v += dt * a
        x += dt * v
        peak = max(peak, x)
    print("  altitude overshoot after launch hand-off at +0.5 m/s: %.3f m" % peak)


pitch_sign = [-1, -1, 1, 1]  # motorPitchSign
yaw_sign = [-1, 1, -1, 1]    # motorYawSign
km = 5.0e-7


def run_coupled(m, Ixx, Iyy, Izz, arm_eff, T=8.0, yaw_kick=0.001):
    """Roll, pitch, yaw rate and altitude through one shared mixer (Flight.cs:546-893).
    Returns (max height gained, mean thrust/hover over the last 1 s, final yaw rate)."""
    th_p = th_r = 0.0
    wp, wy, wr = 0.0, yaw_kick, 0.0
    z = vz = 0.0
    ratios = []
    for _ in range(int(T / dt)):
        mag = math.sqrt(wp * wp + wy * wy + wr * wr)
        if mag > clamp:
            s = clamp / mag
            wp, wy, wr = wp * s, wy * s, wr * s
        hover = m * g
        thrust = hover * (1 + max(-1.8, min(1.8, 0.5 * (0 - z) - 0.8 * vz)))
        tilt = math.cos(th_r) * math.cos(th_p)
        if tilt > 0.01:
            thrust *= min(1 / tilt, 1.15)
        tp = Kp_rp * -th_p - Kd_rp * wp
        tr = Kp_rp * -th_r - Kd_rp * wr
        ty = Kp_yaw * -wy
        F = [thrust / 4 + roll_sign[i] * tr / (4 * arm_eff) + pitch_sign[i] * tp / (4 * arm_eff)
             + yaw_sign[i] * ty * kf / (4 * km) for i in range(4)]
        lo, hi = min(F), max(F)
        if lo < 0:
            F = [f - lo for f in F]
            hi -= lo
        if hi > F_max:
            F = [f - (hi - F_max) for f in F]
        F = [min(max(f, 0.0), F_max) for f in F]
        Tp = sum(pitch_sign[i] * F[i] * arm_eff for i in range(4))
        Tr = sum(roll_sign[i] * F[i] * arm_eff for i in range(4))
        Ty = sum(yaw_sign[i] * km / kf * F[i] for i in range(4))
        wp += dt * (Tp - ang_drag * wp) / Ixx
        wr += dt * (Tr - ang_drag * wr) / Izz
        wy += dt * (Ty - ang_drag * wy) / Iyy
        wp, wr, wy = (x / (1 + dt * ang_damp) for x in (wp, wr, wy))
        mag = math.sqrt(wp * wp + wy * wy + wr * wr)
        if mag > 6.0:
            s = 6.0 / mag
            wp, wy, wr = wp * s, wy * s, wr * s
        th_p += dt * wp
        th_r += dt * wr
        vz += dt * (sum(F) * math.cos(th_r) * math.cos(th_p) - m * g - 0.12 * vz) / m
        z += dt * vz
        ratios.append(sum(F) / hover)
    return z, sum(ratios[-50:]) / 50, wy


def coupled_table():
    print("\nCOUPLED (3 axes + altitude, shared mixer), no input, 8 s, 0.001 rad/s yaw kick")
    cases = [("stock", 2.0, 0.003, 0.006, 0.003, 0.16),
             ("cube quad", 1.7, 0.006167, 0.010667, 0.006167, 0.2121),
             ("1.2 kg yaw-light (Unity: level runaway climb)", 1.2, 0.002167, 0.002667, 0.002167, 0.0707),
             ("0.28 kg small (Unity: runaway climb, crash)", 0.28, 0.000679, 0.001191, 0.000679, 0.1131)]
    for name, m, Ixx, Iyy, Izz, arm in cases:
        z, ratio, wy = run_coupled(m, Ixx, Iyy, Izz, arm * math.sqrt(2) / 2)
        print("  %-46s height after 8 s %+9.2f m, thrust/hover last 1 s %5.2f, yaw rate %+.2f rad/s"
              % (name, z, ratio, wy))


if __name__ == "__main__":
    attitude_table()
    yaw_table()
    limits()
    coupled_table()
