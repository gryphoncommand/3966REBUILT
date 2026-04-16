package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveIO;

public class PositionParameterizedPathCommand extends Command {

    // ── Tuning ────────────────────────────────────────────────────────────────
    private static final double V_MAX           = 5.0;   // m/s
    private static final double A_MAX           = 5.5;   // m/s²
    private static final double A_LATERAL_MAX   = 6.0;   // m/s² (curvature speed limit)
    private static final double K_CROSS         = 6.0;   // cross-track gain  (1/m → m/s)
    private static final double K_ROT           = 8.0;   // robot-facing heading gain
    private static final double OMEGA_MAX       = 10.0;  // rad/s — clamp to prevent wheel saturation
    private static final double SEARCH_WINDOW   = 1.5;   // metres ahead to search for closest point
    private static final double END_THRESHOLD   = 0.5;   // metres — done when this close to end
    private static final double SPEED_LOOKAHEAD = 0.5;   // metres ahead for speed lookup
    private static final double STEER_LOOKAHEAD = 0.6;   // metres ahead for steering target
    private static final double ROT_LOOKAHEAD   = 0.5;   // metres ahead to read rotation target

    private static final Timer runningTimer = new Timer();
    private int maxIndexReached = 0;

    // ── Precomputed path data (immutable after construction) ──────────────────
    private final List<Translation2d> positions;      // dense path positions
    private final double[] arcLengths;                // s[i] — cumulative arc length to point i
    private final double[] tangentHeadings;           // path tangent angle at each point (radians)
    private final double[] curvatures;                // signed curvature κ at each point
    private final double[] velocityProfile;           // v(s) — speed at each point
    private final double[] builtRotationTargets;      // immutable — computed once in constructor
    private final double[] rotationRates;             // dθ/ds at each point
    private boolean hasRotationTargets = false;       // true if path defines any rotation targets

    // ── Runtime mutable copy (safe to overwrite each initialize()) ────────────
    private final double[] rotationTargets;           // working copy of builtRotationTargets

    private final DriveIO drive;

    // ── Runtime state ─────────────────────────────────────────────────────────
    private double currentS = 0.0;

    // ─────────────────────────────────────────────────────────────────────────

    public PositionParameterizedPathCommand(DriveIO drive, PathPlannerPath path) {
        this.drive = drive;

        List<PathPoint> raw = path.getAllPathPoints();
        int n = raw.size();

        positions            = new ArrayList<>(n);
        arcLengths           = new double[n];
        tangentHeadings      = new double[n];
        curvatures           = new double[n];
        velocityProfile      = new double[n];
        builtRotationTargets = new double[n];
        rotationTargets      = new double[n];
        rotationRates        = new double[n];

        for (PathPoint pt : raw) positions.add(pt.position);

        buildArcLengths();
        buildTangentHeadings();
        buildCurvatures();
        buildVelocityProfile();
        buildRotationTargets(raw);           // writes into builtRotationTargets
        buildRotationRates();                // reads builtRotationTargets
        System.arraycopy(builtRotationTargets, 0, rotationTargets, 0, n);

        addRequirements(drive);
    }

    // ── Precomputation ────────────────────────────────────────────────────────

    private void buildArcLengths() {
        arcLengths[0] = 0.0;
        for (int i = 1; i < positions.size(); i++)
            arcLengths[i] = arcLengths[i - 1] + positions.get(i).getDistance(positions.get(i - 1));
    }

    private void buildTangentHeadings() {
        int n = positions.size();
        for (int i = 0; i < n; i++) {
            Translation2d delta;
            if      (i == 0)     delta = positions.get(1).minus(positions.get(0));
            else if (i == n - 1) delta = positions.get(n - 1).minus(positions.get(n - 2));
            else                 delta = positions.get(i + 1).minus(positions.get(i - 1));
            tangentHeadings[i] = Math.atan2(delta.getY(), delta.getX());
        }
    }

    /** Menger curvature from three consecutive points. */
    private void buildCurvatures() {
        int n = positions.size();
        for (int i = 0; i < n; i++) {
            if (i == 0 || i == n - 1) { curvatures[i] = 0.0; continue; }

            Translation2d a = positions.get(i - 1);
            Translation2d b = positions.get(i);
            Translation2d c = positions.get(i + 1);

            double signedArea = 0.5 * ((b.getX() - a.getX()) * (c.getY() - a.getY())
                                     - (c.getX() - a.getX()) * (b.getY() - a.getY()));
            double ab = a.getDistance(b);
            double bc = b.getDistance(c);
            double ca = c.getDistance(a);
            double denom = ab * bc * ca;

            curvatures[i] = (denom < 1e-9) ? 0.0 : (4.0 * signedArea) / denom;
        }
    }

    /**
     * Trapezoidal velocity profile entirely over arc length — no time involved.
     * Three constraints applied per point:
     *   1. Global speed cap V_MAX
     *   2. Lateral-acceleration curvature limit: v ≤ sqrt(A_LATERAL_MAX / |κ|)
     *   3. Forward/backward kinematic accel passes
     */
    private void buildVelocityProfile() {
        int n = positions.size();

        // Curvature-limited ceiling
        for (int i = 0; i < n; i++)
            velocityProfile[i] = Math.min(V_MAX,
                Math.sqrt(A_LATERAL_MAX / Math.max(Math.abs(curvatures[i]), 1e-9)));

        // Start and end at rest
        velocityProfile[0]     = 0.0;
        velocityProfile[n - 1] = 0.0;

        // Forward pass — acceleration limit
        for (int i = 1; i < n; i++) {
            double ds = arcLengths[i] - arcLengths[i - 1];
            velocityProfile[i] = Math.min(velocityProfile[i],
                Math.sqrt(velocityProfile[i - 1] * velocityProfile[i - 1] + 2 * A_MAX * ds));
        }

        // Backward pass — deceleration limit
        for (int i = n - 2; i >= 0; i--) {
            double ds = arcLengths[i + 1] - arcLengths[i];
            velocityProfile[i] = Math.min(velocityProfile[i],
                Math.sqrt(velocityProfile[i + 1] * velocityProfile[i + 1] + 2 * A_MAX * ds));
        }
    }

    /**
     * Walk the PathPoints, pulling rotationTarget values where present and
     * linearly interpolating over arc length between them.
     * Writes into builtRotationTargets (never rotationTargets directly).
     * Sets hasRotationTargets = true when the path defines at least one target.
     */
    private void buildRotationTargets(List<PathPoint> raw) {
        int n = raw.size();

        List<Integer> anchorIdx    = new ArrayList<>();
        List<Double>  anchorAngles = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            PathPoint pt = raw.get(i);
            if (pt.rotationTarget != null) {
                anchorIdx.add(i);
                anchorAngles.add(pt.rotationTarget.rotation().getRadians());
            }
        }

        if (anchorIdx.isEmpty()) {
            // No rotation targets defined — will be filled at initialize()
            hasRotationTargets = false;
            return;
        }

        hasRotationTargets = true;

        // Fill before first target
        for (int i = 0; i <= anchorIdx.get(0); i++)
            builtRotationTargets[i] = anchorAngles.get(0);

        // Interpolate between consecutive targets
        for (int seg = 0; seg < anchorIdx.size() - 1; seg++) {
            int    i0 = anchorIdx.get(seg),    i1 = anchorIdx.get(seg + 1);
            double a0 = anchorAngles.get(seg), a1 = anchorAngles.get(seg + 1);
            for (int i = i0; i <= i1; i++) {
                double t = (double)(i - i0) / (i1 - i0);
                builtRotationTargets[i] = a0 + t * MathUtil.angleModulus(a1 - a0);
            }
        }

        // Fill after last target
        for (int i = anchorIdx.get(anchorIdx.size() - 1); i < n; i++)
            builtRotationTargets[i] = anchorAngles.get(anchorAngles.size() - 1);
    }

    /** dθ/ds at each point — used as rotation feedforward. Reads builtRotationTargets. */
    private void buildRotationRates() {
        int n = builtRotationTargets.length;
        rotationRates[0]     = 0.0;
        rotationRates[n - 1] = 0.0;
        for (int i = 1; i < n - 1; i++) {
            double ds = arcLengths[i + 1] - arcLengths[i - 1];
            if (ds < 1e-9) { rotationRates[i] = 0.0; continue; }
            double dTheta = MathUtil.angleModulus(builtRotationTargets[i + 1] - builtRotationTargets[i - 1]);
            rotationRates[i] = dTheta / ds;
        }
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        Pose2d pose = drive.getCurrentPose();

        // Always restore the immutable precomputed rotation targets first,
        // so repeated runs of the same command object are not affected by
        // the heading-hold overwrite below.
        System.arraycopy(builtRotationTargets, 0, rotationTargets, 0, rotationTargets.length);

        // Seed currentS at the point closest to where the robot currently is
        currentS = 0.0;
        double bestDist = Double.MAX_VALUE;
        int n = positions.size();
        int maxStartIdx = Math.max(1, n / 4); // search first quarter only
        int bestIdx = 0;

        for (int i = 0; i <= maxStartIdx; i++) {
            double d = pose.getTranslation().getDistance(positions.get(i));
            if (d < bestDist) {
                bestDist = d;
                bestIdx  = i;
            }
        }
        currentS = arcLengths[bestIdx];

        // Only overwrite rotation targets when the path genuinely had none.
        // Using an explicit flag avoids treating 0.0 rad (field-east) as "unset".
        if (!hasRotationTargets) {
            double initialAngle = pose.getRotation().getRadians();
            for (int i = 0; i < rotationTargets.length; i++)
                rotationTargets[i] = initialAngle;
        }

        runningTimer.restart();
        maxIndexReached = 0;
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        Pose2d robot = drive.getCurrentPose();

        // ── 1. Project robot onto path ──────────────────────────────────────
        currentS = project(robot.getTranslation());

        int currentIdx = lowerBound(currentS);
        maxIndexReached = Math.max(maxIndexReached, currentIdx);
        Logger.recordOutput("Drive/Max Waypoint Reached", maxIndexReached + "/" + positions.size());

        double lastS = arcLengths[arcLengths.length - 1];

        double speedS = Math.min(currentS + SPEED_LOOKAHEAD, lastS);
        double rotS   = Math.min(currentS + ROT_LOOKAHEAD,   lastS);
        double steerS = Math.min(currentS + STEER_LOOKAHEAD, lastS);

        // ── 2. Interpolate path quantities ──────────────────────────────────
        double refKappa        = interpolate(curvatures,      currentS);
        double refSpeed        = interpolate(velocityProfile, speedS);
        double refRotation     = interpolate(rotationTargets, rotS);
        double refRotationRate = interpolate(rotationRates,   rotS);

        // Lookahead point for steering (reduces lag vs. closest-point steering)
        Translation2d steerPos    = interpolatePosition(steerS);
        double        steerTangent = interpolate(tangentHeadings, steerS);

        Translation2d refPos = interpolatePosition(currentS);
        Logger.recordOutput("Drive/PathFollowGoalPose", new Pose2d(refPos, new Rotation2d(refRotation)));
        Logger.recordOutput("Drive/Path Follow Theta Error",
            new Rotation2d(refRotation).minus(robot.getRotation()).getDegrees());

        // ── 3. Path-frame errors ────────────────────────────────────────────
        double dx    = robot.getX() - steerPos.getX();
        double dy    = robot.getY() - steerPos.getY();
        double sinT  = Math.sin(steerTangent);
        double cosT  = Math.cos(steerTangent);

        double eCross   = -dx * sinT + dy * cosT;   // lateral offset from path (signed)
        double eHeading =  MathUtil.angleModulus(robot.getRotation().getRadians() - refRotation);

        // ── 4. Commands ─────────────────────────────────────────────────────
        double vForward = refSpeed;
        double vLateral = -K_CROSS * eCross;

        // Field-relative translation
        double vx = vForward * cosT - vLateral * sinT;
        double vy = vForward * sinT + vLateral * cosT;

        // Rotation: rate feedforward + curvature feedforward + heading feedback
        double omega = MathUtil.clamp(
            refRotationRate * vForward + vForward * refKappa - K_ROT * eHeading,
            -OMEGA_MAX, OMEGA_MAX);

        drive.driveFieldRelativeChassis(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        int n = positions.size();
        boolean passedHalfway = maxIndexReached >= n / 2;
        boolean nearEnd = drive.getCurrentPose().getTranslation()
                .getDistance(positions.get(n - 1)) < END_THRESHOLD;

        Logger.recordOutput("Drive/Positional Path End Condition",
            "Passed Halfway " + passedHalfway + ", Near End " + nearEnd);

        return passedHalfway && nearEnd;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveFieldRelativeChassis(new ChassisSpeeds());
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Finds the closest point on the path to {@code robotPos}, searching only
     * from currentS forward by a dynamic window. Prevents snapping backward.
     */
    private double project(Translation2d robotPos) {
        ChassisSpeeds spd = drive.getCurrentSpeeds();
        double speed = Math.hypot(spd.vxMetersPerSecond, spd.vyMetersPerSecond);
        double dynamicWindow = Math.max(SEARCH_WINDOW, speed * 0.3);

        int startIdx = lowerBound(currentS);
        int endIdx   = Math.min(lowerBound(currentS + dynamicWindow), positions.size() - 1);

        double bestS    = currentS;
        double bestDist = Double.MAX_VALUE;

        for (int i = startIdx; i <= endIdx; i++) {
            double d = robotPos.getDistance(positions.get(i));
            if (d < bestDist) { bestDist = d; bestS = arcLengths[i]; }
        }
        return bestS;
    }

    /** Binary search: first index where arcLengths[i] >= s. */
    private int lowerBound(double s) {
        int lo = 0, hi = arcLengths.length - 1;
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (arcLengths[mid] < s) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    private double interpolate(double[] values, double s) {
        int i = lowerBound(s);
        if (i == 0) return values[0];
        if (i >= values.length) return values[values.length - 1];
        double t = (s - arcLengths[i - 1]) / (arcLengths[i] - arcLengths[i - 1]);
        return values[i - 1] + t * (values[i] - values[i - 1]);
    }

    private Translation2d interpolatePosition(double s) {
        int i = lowerBound(s);
        if (i == 0) return positions.get(0);
        if (i >= positions.size()) return positions.get(positions.size() - 1);
        double t = (s - arcLengths[i - 1]) / (arcLengths[i] - arcLengths[i - 1]);
        return positions.get(i - 1).interpolate(positions.get(i), t);
    }
}