# ShooterState(distanceMeters, hoodAngleDeg, flywheelRPM, flightTimeSec)

import argparse
import sys
import numpy as np

# Physical & hardware constants

G              = 9.81 # m/s^2
BALL_RADIUS    = 0.150114 / 2 # 0.075057 m
ENTRY_HEIGHT   = 1.8288 # m  (goal lip height)
ENTRY_RADIUS   = 0.56 # m  (FuelSim Hub.ENTRY_RADIUS)

LAUNCH_HEIGHT  = 17.701451 * 0.0254 # Height in meters off the ground
WHEEL_RADIUS   = 2.0 * 0.0254 # Flywheel radius in meters
EFFICIENCY     = 0.7 # Coefficient of shooter efficiency

# Ball comes out at 90 - hood angle, so these are the limits on launch angle from horizontal
HOOD_MIN_DEG   = 22.6
HOOD_MAX_DEG   = 52.5

SHOOTER_OFFSET = 0.260 # m  kRobotToShooter.x

# Air-resistance parameters
AIR_DENSITY    = 1.2041 # kg/m^3
DRAG_COF       = 0.47 # smooth sphere
FUEL_MASS      = 0.448 * 0.45392 # kg
CROSS_AREA     = np.pi * BALL_RADIUS ** 2 # m^2
DRAG_FACTOR    = 0.5 * AIR_DENSITY * DRAG_COF * CROSS_AREA # (scalar) × speed/mass -> accel

#  Unit helpers 

def launch_from_hood(hood_deg: float) -> float:
    """Launch angle from horizontal = 90 - hood angle."""
    return 90.0 - hood_deg

def hood_from_launch(launch_deg: float) -> float:
    return 90.0 - launch_deg

def speed_to_rpm(v: float) -> float:
    return v / (WHEEL_RADIUS * EFFICIENCY) * 60.0 / (2.0 * np.pi)

def rpm_to_speed(rpm: float) -> float:
    return rpm * 2.0 * np.pi / 60.0 * WHEEL_RADIUS * EFFICIENCY

#  Analytical range equation (no drag) 

def analytical_speed(d: float, la_deg: float) -> float | None:
    """
    Closed-form launch speed for a ball fired at `la_deg`° from horizontal,
    starting at z=LAUNCH_HEIGHT, to arrive at (d, ENTRY_HEIGHT).

    Range equation:
        ENTRY_HEIGHT = LAUNCH_HEIGHT + d·tan(a) - g·d^2 / (2v^2·cos^2a)
    Solving for v:
        v^2 = g·d^2 / (2·cos^2a · (LAUNCH_HEIGHT + d·tan(a) - ENTRY_HEIGHT))

    Returns None when the geometry is unreachable.
    """
    a   = np.radians(la_deg)
    dz  = LAUNCH_HEIGHT + d * np.tan(a) - ENTRY_HEIGHT
    if dz <= 0.0:
        return None
    return float(np.sqrt(G * d * d / (2.0 * np.cos(a) ** 2 * dz)))

#  RK4 trajectory integrator 

def integrate(
    v: float,
    la_deg: float,
    air_resistance: bool = True,
    dt: float = 2e-4,
    t_max: float = 5.0,
) -> dict | None:
    """
    Integrate the trajectory with RK4.

    Returns a dict {t, z_peak, x, vz} at the moment the ball first descends
    through z = ENTRY_HEIGHT, or None if it never does so.
    """
    a   = np.radians(la_deg)
    vx  = v * np.cos(a)
    vz  = v * np.sin(a)
    x, z = 0.0, LAUNCH_HEIGHT
    t     = 0.0
    z_pk  = LAUNCH_HEIGHT
    pz    = z

    def accel(vx_: float, vz_: float):
        az = -G
        ax = 0.0
        if air_resistance:
            spd = np.hypot(vx_, vz_)
            if spd > 1e-9:
                k   = DRAG_FACTOR * spd / FUEL_MASS
                ax -= k * vx_
                az -= k * vz_
        return ax, az

    while t < t_max:
        ax0, az0 = accel(vx, vz)
        k1x,  k1z  = vx * dt,        vz * dt
        k1vx, k1vz = ax0 * dt,       az0 * dt

        ax1, az1 = accel(vx + .5*k1vx, vz + .5*k1vz)
        k2x,  k2z  = (vx + .5*k1vx)*dt, (vz + .5*k1vz)*dt
        k2vx, k2vz = ax1*dt, az1*dt

        ax2, az2 = accel(vx + .5*k2vx, vz + .5*k2vz)
        k3x,  k3z  = (vx + .5*k2vx)*dt, (vz + .5*k2vz)*dt
        k3vx, k3vz = ax2*dt, az2*dt

        ax3, az3 = accel(vx + k3vx, vz + k3vz)
        k4x,  k4z  = (vx + k3vx)*dt, (vz + k3vz)*dt
        k4vx, k4vz = ax3*dt, az3*dt

        vx += (k1vx + 2*k2vx + 2*k3vx + k4vx) / 6.0
        vz += (k1vz + 2*k2vz + 2*k3vz + k4vz) / 6.0
        x  += (k1x  + 2*k2x  + 2*k3x  + k4x)  / 6.0
        z  += (k1z  + 2*k2z  + 2*k3z  + k4z)  / 6.0
        t  += dt
        z_pk = max(z_pk, z)

# Descending crossing of ENTRY_HEIGHT
        if pz > ENTRY_HEIGHT >= z and vz < 0:
            return dict(t=t, z_peak=z_pk, x=x, vz=vz)
        pz = z

    return None

#  Numerical speed-finder (needed when air_resistance=True) 

def find_speed(
    la_deg: float,
    shooter_dist: float,
    air_resistance: bool,
    tol: float = 2e-3,
    dt: float = 2e-4,
) -> dict | None:
    """
    Find the launch speed such that the ball enters the hub at `shooter_dist`.

    Without drag:  the analytical formula is exact; RK4 just extracts timing.
    With drag:     drag slows the ball, so we binary-search for a higher speed
                   that overcomes the energy loss.

    Returns a dict with keys {speed, t, z_peak, x, vz}, or None on failure.
    """
    v0 = analytical_speed(shooter_dist, la_deg)
    if v0 is None:
        return None

    if not air_resistance:
        r = integrate(v0, la_deg, air_resistance=True, dt=dt)
        if r is None:
            return None
        r['speed'] = v0
        return r

# ---- with drag: binary search over [v0, 2.5·v0] ----
    v_lo, v_hi = v0, v0 * 2.5
    r_hi = integrate(v_hi, la_deg, air_resistance=True, dt=dt)
    if r_hi is None or r_hi['x'] < shooter_dist:
        return None # can't reach even at high speed

    for _ in range(45 if dt < 1e-3 else 18):
        v_m = (v_lo + v_hi) / 2.0
        r   = integrate(v_m, la_deg, air_resistance=True, dt=dt)
        if r is None:
            v_lo = v_m
            continue
        err = r['x'] - shooter_dist
        if abs(err) < tol:
            r['speed'] = v_m
            return r
        if err < 0:
            v_lo = v_m # ball lands short -> need more power
        else:
            v_hi = v_m # ball lands long  -> reduce power

    return None

#  Main table generator 

def generate(
    robot_distances: list[float],
    air_resistance: bool = False,
    arc: str = 'low',
    n_angles: int = 3000,
) -> list[dict]:
    """
    For each robot-to-hub distance, sweep valid hood angles and choose the shot
    with the lowest peak height ('low' arc) or highest ('high' arc).

    When air_resistance=True a coarser grid is used to keep runtime reasonable.
    """
    la_lo  = launch_from_hood(HOOD_MAX_DEG)
    la_hi  = launch_from_hood(HOOD_MIN_DEG)

# Air-resistance mode: coarser sweep + larger RK4 step
    if air_resistance:
        n_angles = min(n_angles, 80)
        dt = 5e-3 # 5 ms — fast; drag error is small at these ball speeds
    else:
        dt = 2e-4

    angles = np.linspace(la_lo, la_hi, n_angles)

    rows = []

    for rd in robot_distances:
        sd = rd - SHOOTER_OFFSET # horizontal distance: shooter -> hub
        if sd < 0.3:
            print(f"  {rd:.3f} m  [skip — too close after shooter offset]")
            continue

        best: dict | None = None

        for la in angles:
            r = find_speed(la, sd, air_resistance, dt=dt)
            if r is None:
                continue

            # Ball must land within the hub opening (centred on hub)
            if abs(r['x'] - sd) > ENTRY_RADIUS:
                continue
            # Ball must have risen above the goal lip
            if r['z_peak'] <= ENTRY_HEIGHT:
                continue

            prefer = (
                (arc == 'low'  and (best is None or r['z_peak'] < best['z_peak'])) or
                (arc == 'high' and (best is None or r['z_peak'] > best['z_peak']))
            )
            if prefer:
                best = r
                best['la']   = la
                best['hood'] = hood_from_launch(la)

        if best:
            best['robot_dist'] = rd
            best['rpm']        = speed_to_rpm(best['speed'])
            rows.append(best)
            print(
                f"  {rd:.3f} m  ->  "
                f"hood {best['hood']:5.2f}°  "
                f"launch {best['la']:4.1f}°  "
                f"RPM {best['rpm']:6.0f}  "
                f"t {best['t']:.3f} s  "
                f"peak {best['z_peak']:.3f} m"
            )
        else:
            print(f"  {rd:.3f} m  [no valid shot found]")

    return rows

def generate_fixed_hood(
    robot_distances: list[float],
    hood_deg: float = 28.0,
    air_resistance: bool = True,
    dt: float = 1e-5,
) -> list[dict]:
    la = launch_from_hood(hood_deg)
    rows = []

    for rd in robot_distances:
        sd = rd - SHOOTER_OFFSET
        if sd < 0.3:
            print(f"  {rd:.3f} m  [too close after shooter offset]")
            continue

        v0 = analytical_speed(sd, la)
        if v0 is None:
            print(f"  {rd:.3f} m  [no possible shot at hood {hood_deg}°]")
            continue

# Slightly increase to overcome air resistance
        v_guess = v0 * (1.05 if air_resistance else 1.0)
        r = integrate(v_guess, la, air_resistance=air_resistance, dt=dt)
        if r is None:
            print(f"  {rd:.3f} m  [integration failed]")
            continue

        r['speed'] = v_guess
        r['la'] = la
        r['hood'] = hood_deg
        r['robot_dist'] = rd
        r['rpm'] = speed_to_rpm(r['speed'])
        rows.append(r)

        print(
            f"  {rd:.3f} m -> hood {r['hood']:5.2f}° "
            f"launch {r['la']:4.1f}° RPM {r['rpm']:6.0f} "
            f"t {r['t']:.3f}s peak {r['z_peak']:.3f}m"
        )

    return rows
#  Output 

def emit_java(rows: list[dict], arc: str) -> None:
    var = "RealShootingValuesLow" if arc == 'low' else "RealShootingValuesHigh"
    print()
    print(f"public static List<ShooterState> {var} = List.of(")
    for i, r in enumerate(rows):
        sep = "," if i < len(rows) - 1 else ""
        print(
            f"  new ShooterState("
            f"{r['robot_dist']:.6f}, "
            f"{r['hood']:.2f}, "
            f"{r['rpm']:.1f}, "
            f"{r['t']:.2f}){sep}"
        )
    print(");")

def plot_trajectories(rows: list[dict], air_resistance: bool) -> None:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("\n[plot] matplotlib not installed — skipping plot.")
        return

    fig, ax = plt.subplots(figsize=(12, 5))

    cmap = plt.cm.viridis(np.linspace(0, 1, len(rows)))
    labels = []

    for row, colour in zip(rows, cmap):
        la, v, rd = row['la'], row['speed'], row['robot_dist']
        sd = rd - SHOOTER_OFFSET
        a  = np.radians(la)
        vx = v * np.cos(a)

        # Dense sample for smooth curve
        ts  = np.linspace(0, row['t'] * 1.05, 500)
        xs  = vx * ts
        zs  = LAUNCH_HEIGHT + v * np.sin(a) * ts - 0.5 * G * ts ** 2
        # (no drag in plot path for clarity; drag only changes the tuned speed slightly)

        ax.plot(xs, zs, color=colour, lw=1.8)
        ax.plot(row['x'], ENTRY_HEIGHT, 'o', color=colour, ms=5)
        labels.append(mpatches.Patch(color=colour, label=f"{rd:.2f} m"))

    # Hub opening
    ax.fill_betweenx(
        [0, ENTRY_HEIGHT],
        sd - ENTRY_RADIUS, sd + ENTRY_RADIUS,
        alpha=0.15, color='steelblue', label='Hub opening (last dist)'
    )
    ax.axhline(ENTRY_HEIGHT, ls='--', color='steelblue', lw=1, label=f'Goal lip {ENTRY_HEIGHT} m')
    ax.axhline(LAUNCH_HEIGHT, ls=':', color='gray', lw=1, label=f'Launch height {LAUNCH_HEIGHT:.3f} m')

    ax.set_xlabel("Horizontal distance from shooter (m)")
    ax.set_ylabel("Height (m)")
    title = "FuelSim Shooting Trajectories"
    if air_resistance:
        title += " (with air resistance)"
    ax.set_title(title)
    ax.legend(handles=labels + [
        mpatches.Patch(color='steelblue', alpha=0.3, label='Hub opening'),
        mpatches.Patch(color='steelblue', label=f'Goal lip {ENTRY_HEIGHT} m'),
    ], loc='upper left', fontsize=8, ncol=2)
    ax.set_ylim(0, max(r['z_peak'] for r in rows) * 1.15)
    ax.set_xlim(-0.1, max(r['x'] for r in rows) * 1.05)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.show()

#  CLI 

def main():
    ap = argparse.ArgumentParser(description="FuelSim shooting table generator")
    ap.add_argument("--arc",  default="high",  choices=["low", "high"],
                    help="low = flattest arc, high = steepest arc (default: low)")
    ap.add_argument("--air",  action="store_true", default=True,
                    help="account for air resistance (mirrors FuelSim physics)")
    ap.add_argument("--dmin", type=float, default=1.5,  metavar="M",
                    help="minimum robot-to-hub distance in metres (default: 1.5)")
    ap.add_argument("--dmax", type=float, default=5.25, metavar="M",
                    help="maximum robot-to-hub distance in metres (default: 5.25)")
    ap.add_argument("--step", type=float, default=0.5,  metavar="M",
                    help="distance step in metres (default: 0.5)")
    ap.add_argument("--plot", action="store_true", default=True,
                    help="display trajectory plot (requires matplotlib)")
    args = ap.parse_args()

    distances = list(np.arange(args.dmin, args.dmax + 1e-9, args.step))

    print("=" * 60)
    print(f"  Arc:            {args.arc.upper()}")
    print(f"  Air resistance: {args.air}")
    print(f"  Goal height:    {ENTRY_HEIGHT} m")
    print(f"  Ball diameter:  {BALL_RADIUS * 2:.6f} m")
    print(f"  Launch height:  {LAUNCH_HEIGHT:.4f} m")
    print(f"  Distances:      {[round(d,2) for d in distances]} m")
    print("=" * 60)

    rows = generate_fixed_hood(distances, air_resistance=args.air)

    if not rows:
        print("\nNo valid shots generated. Try adjusting --dmin / --dmax / --arc.")
        sys.exit(1)

    emit_java(rows, args.arc)

    if args.plot:
        plot_trajectories(rows, args.air)


if __name__ == "__main__":
    main()