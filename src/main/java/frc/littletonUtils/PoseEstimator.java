// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.littletonUtils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import org.photonvision.EstimatedRobotPose;

public class PoseEstimator {
  private static final double historyLengthSecs = 0.15;

  private Pose2d basePose = new Pose2d();
  private Pose2d latestPose = new Pose2d();
  private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();
  private final Matrix<N3, N1> q = new Matrix<>(Nat.N3(), Nat.N1());

  // --- Swerve-specific fields ---
  private SwerveDriveKinematics kinematics = null;
  private SwerveModulePosition[] lastModulePositions = null;
  private Rotation2d lastGyroAngle = null;

  /**
   * Basic constructor — use {@link #addDriveData(double, Twist2d)} to feed odometry.
   * If you want swerve module delta support, use the other constructor.
   */
  public PoseEstimator(Matrix<N3, N1> stateStdDevs) {
    for (int i = 0; i < 3; ++i) {
      q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }
  }

  /**
   * Swerve-aware constructor.
   *
   * @param stateStdDevs        Standard deviations for [x, y, theta] state noise.
   * @param kinematics          The robot's {@link SwerveDriveKinematics} instance.
   * @param initialPositions    Module positions at robot startup / pose reset.
   * @param initialGyroAngle    Gyro heading at robot startup / pose reset.
   */
  public PoseEstimator(
      Matrix<N3, N1> stateStdDevs,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] initialPositions,
      Rotation2d initialGyroAngle) {
    this(stateStdDevs);
    this.kinematics = kinematics;
    this.lastModulePositions = copyPositions(initialPositions);
    this.lastGyroAngle = initialGyroAngle;
  }

  // ---------------------------------------------------------------------------
  // Public API
  // ---------------------------------------------------------------------------

  /** Returns the latest robot pose based on drive and vision data. */
  public Pose2d getLatestPose() {
    return latestPose;
  }

  /** Resets the odometry to a known pose. */
  public void resetPose(Pose2d pose) {
    basePose = pose;
    updates.clear();
    update();
  }

  /**
   * Resets the odometry to a known pose and simultaneously re-seeds the swerve
   * module baseline so the next {@link #addDriveData(double, SwerveModulePosition[], Rotation2d)}
   * call produces correct deltas.
   */
  public void resetPose(
      Pose2d pose, SwerveModulePosition[] currentPositions, Rotation2d currentGyroAngle) {
    lastModulePositions = copyPositions(currentPositions);
    lastGyroAngle = currentGyroAngle;
    resetPose(pose);
  }

  /**
   * Records a new drive movement expressed as a pre-computed {@link Twist2d}.
   * Use this when you are not using the swerve-module-delta path.
   */
  public void addDriveData(double timestamp, Twist2d twist) {
    updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
    update();
  }

  /**
   * Records a new drive movement using raw swerve module positions and the gyro angle.
   *
   * <p>This is more accurate than the plain {@link Twist2d} overload because:
   * <ul>
   *   <li>Each module's individual distance delta is used, so per-module slip is handled
   *       correctly by the kinematics inverse.</li>
   *   <li>The heading component of the resulting twist is replaced with the gyro-measured
   *       rotation, which is far more accurate than the wheel-integrated heading.</li>
   * </ul>
   *
   * <p>Requires the swerve-aware constructor to have been used, or
   * {@link #setKinematics(SwerveDriveKinematics, SwerveModulePosition[], Rotation2d)} to have
   * been called.
   *
   * @param timestamp       FPGA timestamp (seconds) from {@code Timer.getTimestamp()}.
   * @param modulePositions Current absolute module positions (distance + angle per module).
   * @param gyroAngle       Current gyro heading.
   */
  public void addDriveData(
      double timestamp, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
    if (kinematics == null || lastModulePositions == null || lastGyroAngle == null) {
      throw new IllegalStateException(
          "Call the swerve-aware constructor (or setKinematics()) before using "
              + "addDriveData(timestamp, modulePositions, gyroAngle).");
    }

    // 1. Compute per-module position deltas (distance only; keep current angle for direction).
    SwerveModulePosition[] deltas = new SwerveModulePosition[modulePositions.length];
    for (int i = 0; i < modulePositions.length; i++) {
      deltas[i] =
          new SwerveModulePosition(
              modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
              modulePositions[i].angle);
    }

    // 2. Convert module deltas → chassis Twist2d via inverse kinematics.
    Twist2d twist = kinematics.toTwist2d(deltas);

    // 3. Override the rotational component with the (much more accurate) gyro delta.
    //    This eliminates heading drift caused by wheel slip or unequal traction.
    double gyroDeltaRad = gyroAngle.minus(lastGyroAngle).getRadians();
    twist = new Twist2d(twist.dx, twist.dy, gyroDeltaRad);

    // 4. Persist state for next cycle.
    lastModulePositions = copyPositions(modulePositions);
    lastGyroAngle = gyroAngle;

    // 5. Delegate to the standard path.
    addDriveData(timestamp, twist);
  }

  /**
   * Convenience method to configure swerve kinematics after construction with the basic
   * constructor. Useful if the {@link SwerveDriveKinematics} object is not available at
   * construction time.
   */
  public void setKinematics(
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] initialPositions,
      Rotation2d initialGyroAngle) {
    this.kinematics = kinematics;
    this.lastModulePositions = copyPositions(initialPositions);
    this.lastGyroAngle = initialGyroAngle;
  }

  /** Records a new set of vision updates. */
  public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    for (var timestampedVisionUpdate : visionData) {
      var timestamp = timestampedVisionUpdate.timestamp();
      var visionUpdate =
          new VisionUpdate(timestampedVisionUpdate.pose(), timestampedVisionUpdate.stdDevs());
      insertVisionUpdate(timestamp, visionUpdate);
    }
  }

  /** Records a new set of vision updates from PhotonVision estimated poses. */
  public void addVisionData(List<EstimatedRobotPose> estimatedRobotPoses, Matrix<N3, N1> stdevs) {
    for (var estimatedPose : estimatedRobotPoses) {
      var timestamp = estimatedPose.timestampSeconds;
      var visionUpdate = new VisionUpdate(estimatedPose.estimatedPose.toPose2d(), stdevs);
      insertVisionUpdate(timestamp, visionUpdate);
    }
  }

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  /**
   * Shared logic for inserting a single {@link VisionUpdate} at a given timestamp.
   * Handles both the "slot already exists" and "interpolate a new slot" cases.
   */
  private void insertVisionUpdate(double timestamp, VisionUpdate visionUpdate) {
    if (updates.containsKey(timestamp)) {
      var oldVisionUpdates = updates.get(timestamp).visionUpdates();
      oldVisionUpdates.add(visionUpdate);
      oldVisionUpdates.sort(VisionUpdate.compareDescStdDev);
    } else {
      var prevUpdate = updates.floorEntry(timestamp);
      var nextUpdate = updates.ceilingEntry(timestamp);
      if (prevUpdate == null || nextUpdate == null) {
        return; // Outside the range of existing drive data — discard.
      }

      double alpha =
          (timestamp - prevUpdate.getKey()) / (nextUpdate.getKey() - prevUpdate.getKey());

      // Split the next update's twist into two partial twists.
      var twist0 = GeomUtil.multiply(nextUpdate.getValue().twist(), alpha);
      var twist1 = GeomUtil.multiply(nextUpdate.getValue().twist(), 1.0 - alpha);

      var newVisionUpdates = new ArrayList<VisionUpdate>();
      newVisionUpdates.add(visionUpdate);
      newVisionUpdates.sort(VisionUpdate.compareDescStdDev);

      updates.put(timestamp, new PoseUpdate(twist0, newVisionUpdates));
      updates.put(
          nextUpdate.getKey(),
          new PoseUpdate(twist1, nextUpdate.getValue().visionUpdates()));
    }
  }

  /** Clears old data and calculates the latest pose. */
  private void update() {
    while (updates.size() > 1 && updates.firstKey() < Timer.getTimestamp() - historyLengthSecs) {
      var update = updates.pollFirstEntry();
      basePose = update.getValue().apply(basePose, q);
    }

    latestPose = basePose;
    for (var updateEntry : updates.entrySet()) {
      latestPose = updateEntry.getValue().apply(latestPose, q);
    }
  }

  /** Deep-copies an array of {@link SwerveModulePosition} so stored state is never aliased. */
  private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] positions) {
    SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      copy[i] = new SwerveModulePosition(positions[i].distanceMeters, positions[i].angle);
    }
    return copy;
  }

  // ---------------------------------------------------------------------------
  // Records
  // ---------------------------------------------------------------------------

  private static record PoseUpdate(Twist2d twist, ArrayList<VisionUpdate> visionUpdates) {
    public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {
      var pose = lastPose.exp(twist);

      for (VisionUpdate visionUpdate : visionUpdates) {
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
          r[i] = visionUpdate.stdDevs().get(i, 0) * visionUpdate.stdDevs().get(i, 0);
        }
        for (int row = 0; row < 3; ++row) {
          if (q.get(row, 0) == 0.0) {
            visionK.set(row, row, 0.0);
          } else {
            visionK.set(
                row,
                row,
                q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
          }
        }

        var visionTwist = pose.log(visionUpdate.pose());
        var twistMatrix =
            visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));
        pose =
            pose.exp(
                new Twist2d(
                    twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
      }

      return pose;
    }
  }

  public static record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
    public static final Comparator<VisionUpdate> compareDescStdDev =
        (VisionUpdate a, VisionUpdate b) ->
            -Double.compare(
                a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
                b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
  }

  public static record TimestampedVisionUpdate(
      double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}
}