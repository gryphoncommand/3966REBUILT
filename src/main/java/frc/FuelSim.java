package frc;

// https://github.com/hammerheads5000/FuelSim
//
// Copyright (c) 2026 LordOfFrogs & 6328 Littleton Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.littletonUtils.FieldConstants;
import frc.littletonUtils.HubShiftUtil;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FuelSim {
  protected static final double PERIOD = 0.02;
  protected static final int subticks = 3;
  protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
  // Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
  protected static final double AIR_DENSITY = 1.2041; // kg/m^3
  protected static final double FIELD_COR = 0.3; // coefficient of restitution with the field
  protected static final double FUEL_COR = 0.9; // coefficient of restitution with another fuel
  protected static final double NET_COR = 0.2; // coefficient of restitution with the net
  protected static final double ROBOT_COR = 0.1; // coefficient of restitution with a robot
  protected static final double FUEL_RADIUS = 0.075;
  protected static final double FUEL_RADIUS_SQ = FUEL_RADIUS * FUEL_RADIUS;
  protected static final double FUEL_DIAMETER = 2 * FUEL_RADIUS;
  protected static final double FUEL_DIAMETER_SQ = FUEL_DIAMETER * FUEL_DIAMETER;
  protected static final double FIELD_LENGTH = 16.51;
  protected static final double FIELD_WIDTH = 8.04;
  protected static final double TRENCH_WIDTH = 1.265;
  protected static final double TRENCH_BLOCK_WIDTH = 0.305;
  protected static final double TRENCH_HEIGHT = 0.565;
  protected static final double TRENCH_BAR_HEIGHT = 0.102;
  protected static final double TRENCH_BAR_WIDTH = 0.152;
  protected static final double FRICTION = 1.0; // proportion of horizontal vel to lose per sec while on ground
  protected static final double FUEL_MASS = 0.448 * 0.45392; // kgs
  protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
  // Drag coefficient of smooth sphere:
  // https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
  protected static final double DRAG_COF = 0.47; // dimensionless
  protected static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;
  private static FuelSim instance;

  protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
    new Translation3d(0, 0, 0),
    new Translation3d(3.96, 1.57, 0),
    new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(4.61, 1.57, 0.165),
    new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2,
        FIELD_WIDTH - 1.57,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
    new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
    new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(
        4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  private static final int FIELD_LINE_COUNT = FIELD_XZ_LINE_STARTS.length;
  private static final double[] FIELD_LINE_START_X = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_START_Z = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_END_X = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_END_Z = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_Y_MIN = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_Y_MAX = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_VEC_X = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_VEC_Z = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_LEN_SQ = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_NORMAL_X = new double[FIELD_LINE_COUNT];
  private static final double[] FIELD_LINE_NORMAL_Z = new double[FIELD_LINE_COUNT];

  static {
    for (int i = 0; i < FIELD_LINE_COUNT; i++) {
      Translation3d start = FIELD_XZ_LINE_STARTS[i];
      Translation3d end = FIELD_XZ_LINE_ENDS[i];
      FIELD_LINE_START_X[i] = start.getX();
      FIELD_LINE_START_Z[i] = start.getZ();
      FIELD_LINE_END_X[i] = end.getX();
      FIELD_LINE_END_Z[i] = end.getZ();
      FIELD_LINE_Y_MIN[i] = Math.min(start.getY(), end.getY());
      FIELD_LINE_Y_MAX[i] = Math.max(start.getY(), end.getY());
      double vecX = FIELD_LINE_END_X[i] - FIELD_LINE_START_X[i];
      double vecZ = FIELD_LINE_END_Z[i] - FIELD_LINE_START_Z[i];
      FIELD_LINE_VEC_X[i] = vecX;
      FIELD_LINE_VEC_Z[i] = vecZ;
      double lenSq = vecX * vecX + vecZ * vecZ;
      FIELD_LINE_LEN_SQ[i] = lenSq;
      if (lenSq > 1e-12) {
        double invLen = 1.0 / Math.sqrt(lenSq);
        FIELD_LINE_NORMAL_X[i] = -vecZ * invLen;
        FIELD_LINE_NORMAL_Z[i] = vecX * invLen;
      } else {
        FIELD_LINE_NORMAL_X[i] = 0.0;
        FIELD_LINE_NORMAL_Z[i] = 0.0;
      }
    }
  }

  private static final double[][] TRENCH_RECTS = {
    {
      3.96,
      TRENCH_WIDTH,
      0.0,
      5.18,
      TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
      TRENCH_HEIGHT
    },
    {
      3.96,
      FIELD_WIDTH - 1.57,
      0.0,
      5.18,
      FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
      TRENCH_HEIGHT
    },
    {
      FIELD_LENGTH - 5.18,
      TRENCH_WIDTH,
      0.0,
      FIELD_LENGTH - 3.96,
      TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
      TRENCH_HEIGHT
    },
    {
      FIELD_LENGTH - 5.18,
      FIELD_WIDTH - 1.57,
      0.0,
      FIELD_LENGTH - 3.96,
      FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
      TRENCH_HEIGHT
    },
    {
      4.61 - TRENCH_BAR_WIDTH / 2.0,
      0.0,
      TRENCH_HEIGHT,
      4.61 + TRENCH_BAR_WIDTH / 2.0,
      TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
      TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    },
    {
      4.61 - TRENCH_BAR_WIDTH / 2.0,
      FIELD_WIDTH - 1.57,
      TRENCH_HEIGHT,
      4.61 + TRENCH_BAR_WIDTH / 2.0,
      FIELD_WIDTH,
      TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    },
    {
      FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2.0,
      0.0,
      TRENCH_HEIGHT,
      FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2.0,
      TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
      TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    },
    {
      FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2.0,
      FIELD_WIDTH - 1.57,
      TRENCH_HEIGHT,
      FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2.0,
      FIELD_WIDTH,
      TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    },
  };

  protected static class Fuel {
    private static int nextId = 1;

    protected double x;
    protected double y;
    protected double z;
    protected double vx;
    protected double vy;
    protected double vz;
    protected int id;
    protected int gridCol = -1;
    protected int gridRow = -1;
    protected boolean active = true;

    protected Fuel(Translation3d pos, Translation3d vel) {
      this.x = pos.getX();
      this.y = pos.getY();
      this.z = pos.getZ();
      this.vx = vel.getX();
      this.vy = vel.getY();
      this.vz = vel.getZ();
      this.id = nextId++;
    }

    protected Fuel(Translation3d pos) {
      this(pos, new Translation3d());
    }

    protected void update(boolean simulateAirResistance, double dt) {
      x += vx * dt;
      y += vy * dt;
      z += vz * dt;

      if (z > FUEL_RADIUS) {
        double ax = 0.0;
        double ay = 0.0;
        double az = GRAVITY.getZ();

        if (simulateAirResistance) {
          double speedSq = vx * vx + vy * vy + vz * vz;
          if (speedSq > 1e-12) {
            double speed = Math.sqrt(speedSq);
            double dragAccel = DRAG_FORCE_FACTOR * speedSq / FUEL_MASS;

            ax -= dragAccel * (vx / speed);
            ay -= dragAccel * (vy / speed);
            az -= dragAccel * (vz / speed);
          }
        }

        vx += ax * dt;
        vy += ay * dt;
        vz += az * dt;
      }

      if (z <= FUEL_RADIUS + 0.05) {
        vx = vx * (1 - FRICTION * PERIOD / subticks);
        vy = vy * (1 - FRICTION * PERIOD / subticks);

        // pos = new Translation3d(pos.getX(), pos.getY(), FUEL_RADIUS);
      }

      handleFieldCollisions(dt);
    }

    protected void handleXZLineCollision(int lineIndex) {
      if (y < FIELD_LINE_Y_MIN[lineIndex] || y > FIELD_LINE_Y_MAX[lineIndex]) {
        return; // not within y range
      }

      double vecX = FIELD_LINE_VEC_X[lineIndex];
      double vecZ = FIELD_LINE_VEC_Z[lineIndex];
      double lenSq = FIELD_LINE_LEN_SQ[lineIndex];
      if (lenSq <= 1e-12) {
        return;
      }

      double startX = FIELD_LINE_START_X[lineIndex];
      double startZ = FIELD_LINE_START_Z[lineIndex];
      double t = ((x - startX) * vecX + (z - startZ) * vecZ) / lenSq;
      if (t < 0.0 || t > 1.0) {
        return; // projected point not on line segment
      }

      double projX = startX + t * vecX;
      double projZ = startZ + t * vecZ;
      double dx = x - projX;
      double dz = z - projZ;
      double distSq = dx * dx + dz * dz;
      if (distSq > FUEL_RADIUS_SQ) {
        return; // not intersecting line
      }

      double dist = Math.sqrt(distSq);
      double penetration = FUEL_RADIUS - dist;
      if (penetration > 0.0) {
        x += FIELD_LINE_NORMAL_X[lineIndex] * penetration;
        z += FIELD_LINE_NORMAL_Z[lineIndex] * penetration;
      }

      double vDotN = vx * FIELD_LINE_NORMAL_X[lineIndex] + vz * FIELD_LINE_NORMAL_Z[lineIndex];
      if (vDotN > 0.0) {
        return; // already moving away from line
      }
      vx -= (1 + FIELD_COR) * vDotN * FIELD_LINE_NORMAL_X[lineIndex];
      vz -= (1 + FIELD_COR) * vDotN * FIELD_LINE_NORMAL_Z[lineIndex];
    }

    protected void handleFieldCollisions(double dt) {
      // floor and bumps
      for (int i = 0; i < FIELD_LINE_COUNT; i++) {
        handleXZLineCollision(i);
      }

      // edges
      if (x < FUEL_RADIUS && vx < 0) {
        x += FUEL_RADIUS - x;
        vx += -(1 + FIELD_COR) * vx;
      } else if (x > FIELD_LENGTH - FUEL_RADIUS && vx > 0) {
        x += FIELD_LENGTH - FUEL_RADIUS - x;
        vx += -(1 + FIELD_COR) * vx;
      }

      if (y < FUEL_RADIUS && vy < 0) {
        y += FUEL_RADIUS - y;
        vy += -(1 + FIELD_COR) * vy;
      } else if (y > FIELD_WIDTH - FUEL_RADIUS && vy > 0) {
        y += FIELD_WIDTH - FUEL_RADIUS - y;
        vy += -(1 + FIELD_COR) * vy;
      }

      // hubs
      handleHubCollisions(Hub.BLUE_HUB, dt);
      handleHubCollisions(Hub.RED_HUB, dt);

      handleTrenchCollisions();
    }

    protected void handleHubCollisions(Hub hub, double dt) {
      hub.handleHubInteraction(this, dt);
      hub.fuelCollideSide(this);

      double netCollision = hub.fuelHitNet(this);
      if (netCollision != 0.0) {
        x += netCollision;
        vx = -vx * NET_COR;
        vy = vy * NET_COR;
      }
    }

    protected void handleTrenchCollisions() {
      for (double[] rect : TRENCH_RECTS) {
        fuelCollideRectangle(this, rect[0], rect[1], rect[2], rect[3], rect[4], rect[5]);
      }
    }

    protected void addImpulse(double ix, double iy, double iz) {
      vx += ix;
      vy += iy;
      vz += iz;
    }
  }

  protected static void handleFuelCollision(Fuel a, Fuel b) {
    double nx = a.x - b.x;
    double ny = a.y - b.y;
    double nz = a.z - b.z;
    double distSq = nx * nx + ny * ny + nz * nz;
    if (distSq <= 1e-12) {
      nx = 1.0;
      ny = 0.0;
      nz = 0.0;
      distSq = 1.0;
    }
    double distance = Math.sqrt(distSq);
    double invDist = 1.0 / distance;
    nx *= invDist;
    ny *= invDist;
    nz *= invDist;

    double relVx = b.vx - a.vx;
    double relVy = b.vy - a.vy;
    double relVz = b.vz - a.vz;
    double impulse = 0.5 * (1 + FUEL_COR) * (relVx * nx + relVy * ny + relVz * nz);

    double intersection = FUEL_DIAMETER - distance;
    double offset = intersection * 0.5;
    a.x += nx * offset;
    a.y += ny * offset;
    a.z += nz * offset;
    b.x -= nx * offset;
    b.y -= ny * offset;
    b.z -= nz * offset;

    a.addImpulse(nx * impulse, ny * impulse, nz * impulse);
    b.addImpulse(-nx * impulse, -ny * impulse, -nz * impulse);
  }

  protected static final double CELL_SIZE = 0.05;
  protected static final double INV_CELL_SIZE = 1.0 / CELL_SIZE;
  protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
  protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

  @SuppressWarnings("unchecked")
  protected final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

  private final ArrayList<ArrayList<Fuel>> activeCells = new ArrayList<>();
  private static final double STASH_X = 0.0;
  private static final double STASH_Y = 0.0;
  private static final double STASH_Z = -1000.0;
  private final ArrayDeque<Fuel> inactiveFuels = new ArrayDeque<>();
  private int activeFuelCount = 0;
  private int spawnDropCounter = 0;

  protected void handleFuelCollisions(ArrayList<Fuel> fuels) {
    // Clear grid
    for (ArrayList<Fuel> cell : activeCells) {
      cell.clear();
    }
    activeCells.clear();

    // Populate grid
    for (int f = 0, size = fuels.size(); f < size; f++) {
      Fuel fuel = fuels.get(f);
      if (!fuel.active) {
        continue;
      }
      int col = (int) (fuel.x * INV_CELL_SIZE);
      int row = (int) (fuel.y * INV_CELL_SIZE);
      fuel.gridCol = col;
      fuel.gridRow = row;
      if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
        ArrayList<Fuel> cell = grid[col][row];
        cell.add(fuel);
        if (cell.size() == 1) {
          activeCells.add(cell);
        }
      }
    }

    // Check collisions
    for (int f = 0, size = fuels.size(); f < size; f++) {
      Fuel fuel = fuels.get(f);
      if (!fuel.active) {
        continue;
      }
      int col = fuel.gridCol;
      int row = fuel.gridRow;
      // Check 3x3 neighbor cells
      for (int i = col - 1; i <= col + 1; i++) {
        for (int j = row - 1; j <= row + 1; j++) {
          if (i >= 0 && i < GRID_COLS && j >= 0 && j < GRID_ROWS) {
            ArrayList<Fuel> cell = grid[i][j];
            for (int c = 0, cellSize = cell.size(); c < cellSize; c++) {
              Fuel other = cell.get(c);
              if (fuel == other) {
                continue;
              }
              double dx = fuel.x - other.x;
              double dy = fuel.y - other.y;
              double dz = fuel.z - other.z;
              if (dx * dx + dy * dy + dz * dz < FUEL_DIAMETER_SQ) {
                if (fuel.id < other.id) {
                  handleFuelCollision(fuel, other);
                }
              }
            }
          }
        }
      }
    }
  }

  protected ArrayList<Fuel> fuels = new ArrayList<>(512);
  protected boolean running = false;
  protected boolean simulateAirResistance = false;
  protected Supplier<Pose2d> robotPoseSupplier = null;
  protected Supplier<ChassisSpeeds> robotFieldSpeedsSupplier = null;
  protected double robotWidth; // size along the robot's y axis
  protected double robotLength; // size along the robot's x axis
  protected double bumperHeight;
  protected ArrayList<SimIntake> intakes = new ArrayList<>();
  private int logEveryNTicks = 1;
  private int logTickCounter = 0;
  private boolean profilingEnabled = false;
  private int profileEveryNTicks = 50;
  private int profileTickCounter = 0;
  private static final Pose3d[] EMPTY_POSE_ARRAY = new Pose3d[0];
  private Pose3d[] fuelPoseBuffer = EMPTY_POSE_ARRAY;

  /**
   * Creates a new instance of FuelSim
   *
   * @param tableKey NetworkTable to log fuel positions to as an array of {@link Translation3d}
   *     structs.
   */
  public FuelSim(String tableKey) {
    // Initialize grid
    for (int i = 0; i < GRID_COLS; i++) {
      for (int j = 0; j < GRID_ROWS; j++) {
        grid[i][j] = new ArrayList<Fuel>();
      }
    }
  }

  /** Creates a new instance of FuelSim with log path "/Fuel Simulation" */
  public FuelSim() {
    this("/Fuel Simulation");
  }

  /** Clears the field of fuel */
  public void clearFuel() {
    fuels.clear();
    inactiveFuels.clear();
    activeFuelCount = 0;
  }

  private void addFuel(Fuel fuel) {
    fuels.add(fuel);
    activeFuelCount++;
  }

  private void activateFuel(Fuel fuel, Translation3d pos, Translation3d vel) {
    fuel.x = pos.getX();
    fuel.y = pos.getY();
    fuel.z = pos.getZ();
    fuel.vx = vel.getX();
    fuel.vy = vel.getY();
    fuel.vz = vel.getZ();
    fuel.gridCol = -1;
    fuel.gridRow = -1;
    if (!fuel.active) {
      fuel.active = true;
      activeFuelCount++;
    }
  }

  private void deactivateFuel(Fuel fuel) {
    if (!fuel.active) {
      return;
    }
    fuel.active = false;
    fuel.x = STASH_X;
    fuel.y = STASH_Y;
    fuel.z = STASH_Z;
    fuel.vx = 0.0;
    fuel.vy = 0.0;
    fuel.vz = 0.0;
    fuel.gridCol = -1;
    fuel.gridRow = -1;
    activeFuelCount = Math.max(0, activeFuelCount - 1);
    inactiveFuels.addLast(fuel);
  }

  /** Reserve fuels for robot storage (zero-sum sim). Returns count actually reserved. */
  public int reserveFuelForRobot(int count) {
    int reserved = 0;
    for (int i = fuels.size() - 1; i >= 0 && reserved < count; i--) {
      Fuel fuel = new Fuel(new Translation3d());
      if (fuel.active) {
        addFuel(fuel);
        deactivateFuel(fuel);
        reserved++;
      }
    }
    Logger.recordOutput("FuelSim/ReservedFuel", reserved);
    return reserved;
  }

  /** Spawns fuel in the neutral zone and depots */
  public void spawnStartingFuel() {
    // Center fuel
    Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
    for (int i = 0; i < 15; i++) {
      for (int j = 0; j < 6; j++) {
        addFuel(
            new Fuel(
                center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
        addFuel(
            new Fuel(
                center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
        addFuel(
            new Fuel(
                center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
        addFuel(
            new Fuel(
                center.plus(
                    new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
      }
    }

    // Depots
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        addFuel(
            new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
        addFuel(
            new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
        addFuel(
            new Fuel(
                new Translation3d(
                    FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS)));
        addFuel(
            new Fuel(
                new Translation3d(
                    FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS)));
      }
    }

    for (int i = 0; i < 8+24; i++){
      Fuel fuel = new Fuel(new Translation3d());
      addFuel(fuel);
      deactivateFuel(fuel);
    }
  }

  /** Spawns 24 fuels at the outpost opening in waves of 5 across its width. */
  public void spawnOutpostFuel() {
    final int totalFuel = 24;
    final int waveSize = 5;
    final double centerX = FieldConstants.Outpost.centerPoint.getX();
    final double centerY = FieldConstants.Outpost.centerPoint.getY();
    final double minY = centerY - (FieldConstants.Outpost.width / 2.0) + FUEL_RADIUS;
    final double maxY = centerY + (FieldConstants.Outpost.width / 2.0) - FUEL_RADIUS;
    final double baseX = centerX + FUEL_RADIUS;
    final double waveSpacingX = FUEL_DIAMETER;
    final double z = FieldConstants.Outpost.openingDistanceFromFloor;

    for (int i = 0; i < totalFuel; i++) {
      int wave = i / waveSize;
      int indexInWave = i % waveSize;
      int waveCount = Math.min(waveSize, totalFuel - wave * waveSize);
      double yStep = waveCount > 1 ? (maxY - minY) / (waveCount - 1) : 0.0;
      double x = baseX + wave * waveSpacingX;
      double y = minY + indexInWave * yStep;
      spawnFuelIfAvailable(new Translation3d(x, y, z), new Translation3d(Math.random()/2 + 0.2, Math.random() + 0.1, 0));
    }
  }

  /** Adds array of `Translation3d`'s to NetworkTables at tableKey + "/Fuels" */
  public void logFuels() {
    int count = activeFuelCount;
    if (count == 0) {
      Logger.recordOutput("FuelSim/FuelPoses", EMPTY_POSE_ARRAY);
      return;
    }

    if (fuelPoseBuffer.length != count) {
      fuelPoseBuffer = new Pose3d[count];
    }

    int index = 0;
    for (int i = 0, size = fuels.size(); i < size; i++) {
      Fuel fuel = fuels.get(i);
      if (!fuel.active) {
        continue;
      }
      fuelPoseBuffer[index] = new Pose3d(fuel.x, fuel.y, fuel.z, Rotation3d.kZero);
      index++;
    }
    Logger.recordOutput("FuelSim/FuelPoses", fuelPoseBuffer);
  }

  /** Returns the list of fuels in the simulation */
  public Set<Translation2d> getFuels() {
    Set<Translation2d> result = new java.util.HashSet<>(Math.max(16, activeFuelCount * 2));
    for (int f = 0, size = fuels.size(); f < size; f++) {
      Fuel fuel = fuels.get(f);
      if (!fuel.active) {
        continue;
      }
      result.add(new Translation2d(fuel.x, fuel.y));
    }
    return result;
  }

  /** Start the simulation. `updateSim` must still be called every loop */
  public void start() {
    running = true;
  }

  /** Pause the simulation. */
  public void stop() {
    running = false;
  }

  /** Enables accounting for drag force in physics step * */
  public void enableAirResistance() {
    simulateAirResistance = true;
  }

  /** Set how often fuel poses are logged (in sim ticks). */
  public void setLogEveryNTicks(int ticks) {
    logEveryNTicks = Math.max(1, ticks);
  }

  /** Enable lightweight profiling output every N ticks. */
  public void enableProfiling(int everyNTicks) {
    profilingEnabled = true;
    profileEveryNTicks = Math.max(1, everyNTicks);
  }

  /** Disable profiling output. */
  public void disableProfiling() {
    profilingEnabled = false;
  }

  /**
   * Registers a robot with the fuel simulator
   *
   * @param width from left to right (y-axis)
   * @param length from front to back (x-axis)
   * @param bumperHeight
   * @param poseSupplier
   * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
   */
  public void registerRobot(
      double width,
      double length,
      double bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.robotPoseSupplier = poseSupplier;
    this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
    this.robotWidth = width;
    this.robotLength = length;
    this.bumperHeight = bumperHeight;
  }

  /**
   * Registers a robot with the fuel simulator
   *
   * @param width from left to right (y-axis)
   * @param length from front to back (x-axis)
   * @param bumperHeight from the ground
   * @param poseSupplier
   * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
   */
  public void registerRobot(
      Distance width,
      Distance length,
      Distance bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.robotPoseSupplier = poseSupplier;
    this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
    this.robotWidth = width.in(Meters);
    this.robotLength = length.in(Meters);
    this.bumperHeight = bumperHeight.in(Meters);
  }

  /** To be called periodically Will do nothing if sim is not running */
  public void updateSim() {
    if (!running) return;
    stepSim();
    Logger.recordOutput("Basics/Red Scored Fuel", Hub.RED_HUB.getScore());
    Logger.recordOutput("Basics/Blue Scored Fuel", Hub.BLUE_HUB.getScore());
  
  }

  /** Run the simulation forward 1 time step (0.02s) */
  public void stepSim() {
    int effectiveSubticks = subticks;
    double dt = PERIOD / effectiveSubticks;
    boolean doProfile = profilingEnabled && (++profileTickCounter >= profileEveryNTicks);
    if (doProfile) {
      profileTickCounter = 0;
    }
    long tStart = 0L;
    long updateNs = 0L;
    long collisionNs = 0L;
    long robotNs = 0L;
    long intakeNs = 0L;
    if (doProfile) {
      tStart = System.nanoTime();
    }

    for (int i = 0; i < effectiveSubticks; i++) {
      long t0 = 0L;
      long t1 = 0L;
      long t2 = 0L;
      long t3 = 0L;
      if (doProfile) {
        t0 = System.nanoTime();
      }

      for (int f = 0, size = fuels.size(); f < size; f++) {
        Fuel fuel = fuels.get(f);
        if (!fuel.active) {
          continue;
        }
        fuel.update(this.simulateAirResistance, dt);
      }

      if (doProfile) {
        t1 = System.nanoTime();
      }

      handleFuelCollisions(fuels);

      if (doProfile) {
        t2 = System.nanoTime();
      }

      if (robotPoseSupplier != null) {
        handleRobotCollisions(fuels);
        if (doProfile) {
          t3 = System.nanoTime();
        }
        handleIntakes(fuels);
        if (doProfile) {
          updateNs += (t1 - t0);
          collisionNs += (t2 - t1);
          robotNs += (t3 - t2);
          intakeNs += (System.nanoTime() - t3);
        }
      } else if (doProfile) {
        updateNs += (t1 - t0);
        collisionNs += (t2 - t1);
      }
    }

    if (++logTickCounter >= logEveryNTicks) {
      logTickCounter = 0;
      logFuels();
    }

    if (doProfile) {
      long tEnd = System.nanoTime();
      Logger.recordOutput("FuelSim/Profile/FuelCount", activeFuelCount);
      Logger.recordOutput("FuelSim/Profile/Subticks", effectiveSubticks);
      Logger.recordOutput("FuelSim/Profile/UpdateMs", updateNs / 1e6);
      Logger.recordOutput("FuelSim/Profile/CollisionsMs", collisionNs / 1e6);
      Logger.recordOutput("FuelSim/Profile/RobotMs", robotNs / 1e6);
      Logger.recordOutput("FuelSim/Profile/IntakesMs", intakeNs / 1e6);
      Logger.recordOutput("FuelSim/Profile/LogMs", (tEnd - tStart - updateNs - collisionNs - robotNs - intakeNs) / 1e6);
      Logger.recordOutput("FuelSim/Profile/TotalMs", (tEnd - tStart) / 1e6);
    }
  }

  /**
   * Adds a fuel onto the field
   *
   * @param pos Position to spawn at
   * @param vel Initial velocity vector
   */
  public void spawnFuel(Translation3d pos, Translation3d vel) {
    spawnFuelIfAvailable(pos, vel);
  }

  /** Attempts to spawn a fuel from the pool. */
  public boolean spawnFuelIfAvailable(Translation3d pos, Translation3d vel) {
    Fuel fuel = inactiveFuels.pollFirst();
    if (fuel == null) {
      spawnDropCounter++;
      Logger.recordOutput("FuelSim/SpawnDropped", spawnDropCounter);
      return false;
    }
    activateFuel(fuel, pos, vel);
    return true;
  }

  /**
   * Spawns a fuel onto the field with a specified launch velocity and angles, accounting for robot
   * movement
   *
   * @param launchVelocity Initial launch velocity
   * @param hoodAngle Hood angle where 0 is launching horizontally and 90 degrees is launching
   *     straight up
   * @param turretYaw <i>Robot-relative</i> turret yaw
   * @param launchHeight Height of the fuel to launch at. Make sure this is higher than your robot's
   *     bumper height, or else it will collide with your robot immediately.
   * @throws IllegalStateException if robot is not registered
   */
  public void launchFuel(
      LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight) {
    if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null) {
      throw new IllegalStateException("Robot must be registered before launching fuel.");
    }

    Pose3d launchPose =
        new Pose3d(this.robotPoseSupplier.get())
            .plus(
                new Transform3d(
                    new Translation3d(Meters.zero(), Meters.zero(), launchHeight),
                    Rotation3d.kZero));
    ChassisSpeeds fieldSpeeds = this.robotFieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    double verticalVel = Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    double xVel =
        horizontalVel
            * Math.cos(turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));
    double yVel =
        horizontalVel
            * Math.sin(turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
  }

  /**
   * Collects (removes from the simulation) the nearest active fuel within {@code radius} meters of
   * {@code position}. Returns {@code true} if a fuel was found and collected, {@code false} if none
   * was in range. The collected fuel is returned to the inactive pool so it can be re-spawned later
   * (e.g. via {@link #shootFuelIntoRedHub()}).
   */
  public boolean collectFuelAt(Translation2d position, double radius) {
    double radiusSq = radius * radius;
    double px = position.getX();
    double py = position.getY();
    Fuel nearest = null;
    double nearestDistSq = Double.MAX_VALUE;

    for (int i = 0, size = fuels.size(); i < size; i++) {
      Fuel fuel = fuels.get(i);
      if (!fuel.active) continue;
      double dx = fuel.x - px;
      double dy = fuel.y - py;
      double distSq = dx * dx + dy * dy;
      if (distSq < radiusSq && distSq < nearestDistSq) {
        nearest = fuel;
        nearestDistSq = distSq;
      }
    }

    if (nearest != null) {
      deactivateFuel(nearest);
      return true;
    }
    return false;
  }

  /**
   * Shoots one fuel from the inactive pool directly into the red hub's scoring zone. The fuel is
   * spawned just above the entry height with enough downward velocity to cross the threshold in the
   * first physics sub-step, guaranteeing a score. Returns {@code true} if a fuel was available.
   *
   * <p>Red hub center: ({@code FIELD_LENGTH - 4.61}, {@code FIELD_WIDTH / 2}) ≈ (11.90, 4.02)
   * Entry height: 1.83 m, entry radius: 0.56 m.
   */
  public boolean shootFuelIntoRedHub() {
    // Small random spread within 0.2 m of hub center to look natural
    double angle = Math.random() * Math.PI * 2.0;
    double r = Math.random() * 0.2;
    double cx = FIELD_LENGTH - 4.61;
    double cy = FIELD_WIDTH / 2.0;
    Translation3d pos = new Translation3d(
        cx + r * Math.cos(angle),
        cy + r * Math.sin(angle),
        Hub.ENTRY_HEIGHT + 0.1   // just above the scoring threshold
    );
    // vz of -30 m/s clears the threshold in the first sub-tick (dt ≈ 0.004 s)
    return spawnFuelIfAvailable(pos, new Translation3d(0, 0, -30));
  }

  protected void handleRobotCollision(
      Fuel fuel,
      double robotX,
      double robotY,
      double cos,
      double sin,
      double robotVelX,
      double robotVelY) {
    if (fuel.z > bumperHeight) return; // above bumpers

    double dx = fuel.x - robotX;
    double dy = fuel.y - robotY;
    // Rotate into robot frame (field -> robot): rotate by -theta.
    double relativeX = dx * cos + dy * sin;
    double relativeY = -dx * sin + dy * cos;

    double distanceToBottom = -FUEL_RADIUS - robotLength / 2 - relativeX;
    double distanceToTop = -FUEL_RADIUS - robotLength / 2 + relativeX;
    double distanceToRight = -FUEL_RADIUS - robotWidth / 2 - relativeY;
    double distanceToLeft = -FUEL_RADIUS - robotWidth / 2 + relativeY;

    // not inside robot
    if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0)
      return;

    double offsetXRobot;
    double offsetYRobot;
    // find minimum distance to side and send corresponding collision response
    if (distanceToBottom >= distanceToTop
        && distanceToBottom >= distanceToRight
        && distanceToBottom >= distanceToLeft) {
      offsetXRobot = distanceToBottom;
      offsetYRobot = 0.0;
    } else if (distanceToTop >= distanceToBottom
        && distanceToTop >= distanceToRight
        && distanceToTop >= distanceToLeft) {
      offsetXRobot = -distanceToTop;
      offsetYRobot = 0.0;
    } else if (distanceToRight >= distanceToBottom
        && distanceToRight >= distanceToTop
        && distanceToRight >= distanceToLeft) {
      offsetXRobot = 0.0;
      offsetYRobot = distanceToRight;
    } else {
      offsetXRobot = 0.0;
      offsetYRobot = -distanceToLeft;
    }

    // Rotate offset back into field frame (robot -> field)
    double offsetXField = offsetXRobot * cos - offsetYRobot * sin;
    double offsetYField = offsetXRobot * sin + offsetYRobot * cos;
    fuel.x += offsetXField;
    fuel.y += offsetYField;

    double normalMag = Math.hypot(offsetXField, offsetYField);
    if (normalMag > 1e-9) {
      double normalX = offsetXField / normalMag;
      double normalY = offsetYField / normalMag;
      double velDot = fuel.vx * normalX + fuel.vy * normalY;
      if (velDot < 0) {
        double impulse = -velDot * (1 + ROBOT_COR);
        fuel.addImpulse(normalX * impulse, normalY * impulse, 0.0);
      }
      double robotVelDot = robotVelX * normalX + robotVelY * normalY;
      if (robotVelDot > 0) {
        fuel.addImpulse(normalX * robotVelDot, normalY * robotVelDot, 0.0);
      }
    }
  }

  protected void handleRobotCollisions(ArrayList<Fuel> fuels) {
    Pose2d robot = robotPoseSupplier.get();
    ChassisSpeeds speeds = robotFieldSpeedsSupplier.get();
    double robotX = robot.getX();
    double robotY = robot.getY();
    double theta = robot.getRotation().getRadians();
    double cos = Math.cos(theta);
    double sin = Math.sin(theta);
    double robotVelX = speeds.vxMetersPerSecond;
    double robotVelY = speeds.vyMetersPerSecond;

    for (int f = 0, size = fuels.size(); f < size; f++) {
      Fuel fuel = fuels.get(f);
      if (!fuel.active) {
        continue;
      }
      handleRobotCollision(fuel, robotX, robotY, cos, sin, robotVelX, robotVelY);
    }
  }

  protected void handleIntakes(ArrayList<Fuel> fuels) {
    Pose2d robot = robotPoseSupplier.get();
    double robotX = robot.getX();
    double robotY = robot.getY();
    double theta = robot.getRotation().getRadians();
    double cos = Math.cos(theta);
    double sin = Math.sin(theta);

    for (SimIntake intake : intakes) {
      boolean intakeEnabled = intake.ableToIntake.getAsBoolean();
      if (!intakeEnabled) {
        continue;
      }
      for (int i = fuels.size() - 1; i >= 0; i--) {
        Fuel fuel = fuels.get(i);
        if (!fuel.active) {
          continue;
        }
        if (intake.shouldIntake(fuel, robotX, robotY, cos, sin, intakeEnabled)) {
          deactivateFuel(fuel);
        }
      }
    }
  }

  protected static void fuelCollideRectangle(
      Fuel fuel,
      double minX,
      double minY,
      double minZ,
      double maxX,
      double maxY,
      double maxZ) {
    if (fuel.z > maxZ + FUEL_RADIUS || fuel.z < minZ - FUEL_RADIUS) {
      return; // above rectangle
    }
    double distanceToLeft = minX - FUEL_RADIUS - fuel.x;
    double distanceToRight = fuel.x - maxX - FUEL_RADIUS;
    double distanceToTop = fuel.y - maxY - FUEL_RADIUS;
    double distanceToBottom = minY - FUEL_RADIUS - fuel.y;

    // not inside hub
    if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0)
      return;

    double collisionX = 0.0;
    double collisionY = 0.0;
    // find minimum distance to side and send corresponding collision response
    if (fuel.x < minX
        || (distanceToLeft >= distanceToRight
            && distanceToLeft >= distanceToTop
            && distanceToLeft >= distanceToBottom)) {
      collisionX = distanceToLeft;
    } else if (fuel.x >= maxX
        || (distanceToRight >= distanceToLeft
            && distanceToRight >= distanceToTop
            && distanceToRight >= distanceToBottom)) {
      collisionX = -distanceToRight;
    } else if (fuel.y > maxY
        || (distanceToTop >= distanceToLeft
            && distanceToTop >= distanceToRight
            && distanceToTop >= distanceToBottom)) {
      collisionY = -distanceToTop;
    } else {
      collisionY = distanceToBottom;
    }

    if (collisionX != 0.0) {
      fuel.x += collisionX;
      fuel.vx += -(1 + FIELD_COR) * fuel.vx;
    } else if (collisionY != 0.0) {
      fuel.y += collisionY;
      fuel.vy += -(1 + FIELD_COR) * fuel.vy;
    }
  }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based
   * on the `ableToIntake` parameter.
   */
  public void registerIntake(
      double xMin,
      double xMax,
      double yMin,
      double yMax,
      BooleanSupplier ableToIntake,
      Runnable intakeCallback) {
    intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
  }

  public void registerIntake(
      double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
    registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {});
  }

  public void registerIntake(
      double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
    registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
  }

  public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
    registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {});
  }

  public void registerIntake(
      Distance xMin,
      Distance xMax,
      Distance yMin,
      Distance yMax,
      BooleanSupplier ableToIntake,
      Runnable intakeCallback) {
    registerIntake(
        xMin.in(Meters),
        xMax.in(Meters),
        yMin.in(Meters),
        yMax.in(Meters),
        ableToIntake,
        intakeCallback);
  }

  public void registerIntake(
      Distance xMin, Distance xMax, Distance yMin, Distance yMax, BooleanSupplier ableToIntake) {
    registerIntake(
        xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake);
  }

  public void registerIntake(
      Distance xMin, Distance xMax, Distance yMin, Distance yMax, Runnable intakeCallback) {
    registerIntake(
        xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), intakeCallback);
  }

  public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
    registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
  }

  public static class Hub {
    public static final Hub BLUE_HUB =
        new Hub(
            new Translation2d(4.61, FIELD_WIDTH / 2),
            new Translation3d(5.3, FIELD_WIDTH / 2, 0.89),
            1);
    public static final Hub RED_HUB =
        new Hub(
            new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
            new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
            -1);

    protected static final double ENTRY_HEIGHT = 1.83;
    protected static final double ENTRY_RADIUS = 0.56;
    protected static final double ENTRY_RADIUS_SQ = ENTRY_RADIUS * ENTRY_RADIUS;

    protected static final double SIDE = 1.2;

    protected static final double NET_HEIGHT_MAX = 3.057;
    protected static final double NET_HEIGHT_MIN = 1.5;
    protected static final double NET_OFFSET = SIDE / 2 + 0.261;
    protected static final double NET_WIDTH = 1.484;

    protected final Translation2d center;
    protected final Translation3d exit;
    protected final int exitVelXMult;
    private final double centerX;
    private final double centerY;
    private final double exitX;
    private final double exitY;
    private final double exitZ;
    private final double sideMinX;
    private final double sideMaxX;
    private final double sideMinY;
    private final double sideMaxY;
    private final double sideMinZ;
    private final double sideMaxZ;

    protected int score = 0;

    protected Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
      this.center = center;
      this.exit = exit;
      this.exitVelXMult = exitVelXMult;
      this.centerX = center.getX();
      this.centerY = center.getY();
      this.exitX = exit.getX();
      this.exitY = exit.getY();
      this.exitZ = exit.getZ();
      this.sideMinX = centerX - SIDE / 2.0;
      this.sideMaxX = centerX + SIDE / 2.0;
      this.sideMinY = centerY - SIDE / 2.0;
      this.sideMaxY = centerY + SIDE / 2.0;
      this.sideMinZ = 0.0;
      this.sideMaxZ = ENTRY_HEIGHT - 0.1;
    }

    protected void handleHubInteraction(Fuel fuel, double dt) {
      if (didFuelScore(fuel, dt)) {
        fuel.x = exitX;
        fuel.y = exitY;
        fuel.z = exitZ;
        applyDispersalVelocity(fuel);
        Alliance hubAlly = this.equals(Hub.RED_HUB) ? Alliance.Red : Alliance.Blue;
        Alliance DSAlly = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        if (hubAlly.equals(DSAlly) ? HubShiftUtil.getShiftedShiftInfo().active() : HubShiftUtil.isOpposingHubActive()){
          score++;
        }
      }
    }

    protected boolean didFuelScore(Fuel fuel, double dt) {
      double dx = fuel.x - centerX;
      double dy = fuel.y - centerY;
      if (dx * dx + dy * dy > ENTRY_RADIUS_SQ) {
        return false;
      }
      if (fuel.z > ENTRY_HEIGHT) {
        return false;
      }
      double prevZ = fuel.z - fuel.vz * dt;
      return prevZ > ENTRY_HEIGHT;
    }

    protected void applyDispersalVelocity(Fuel fuel) {
      fuel.vx = exitVelXMult * (Math.random() + 0.1) * 1.5;
      fuel.vy = Math.random() * 2 - 1;
      fuel.vz = 0.0;
    }

    /** Reset this hub's score to 0 */
    public void resetScore() {
      score = 0;
    }

    /**
     * Get the current count of fuel scored in this hub
     */
    public int getScore() {
      return score;
    }

    protected void fuelCollideSide(Fuel fuel) {
      fuelCollideRectangle(fuel, sideMinX, sideMinY, sideMinZ, sideMaxX, sideMaxY, sideMaxZ);
    }

    protected double fuelHitNet(Fuel fuel) {
      if (fuel.z > NET_HEIGHT_MAX || fuel.z < NET_HEIGHT_MIN) return 0;
      if (fuel.y > centerY + NET_WIDTH / 2 || fuel.y < centerY - NET_WIDTH / 2) return 0;
      double netX = centerX + NET_OFFSET * exitVelXMult;
      if (fuel.x > netX) {
        return Math.max(
            0, netX - (fuel.x - FUEL_RADIUS));
      } else {
        return Math.min(
            0, netX - (fuel.x + FUEL_RADIUS));
      }
    }
  }

  protected class SimIntake {
    double xMin, xMax, yMin, yMax;
    BooleanSupplier ableToIntake;
    Runnable callback;

    protected SimIntake(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        BooleanSupplier ableToIntake,
        Runnable intakeCallback) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
      this.ableToIntake = ableToIntake;
      this.callback = intakeCallback;
    }

    protected boolean shouldIntake(
        Fuel fuel, double robotX, double robotY, double cos, double sin, boolean intakeEnabled) {
      if (!intakeEnabled || fuel.z > bumperHeight) return false;

      double dx = fuel.x - robotX;
      double dy = fuel.y - robotY;
      // Rotate into robot frame (field -> robot)
      double relativeX = dx * cos + dy * sin;
      double relativeY = -dx * sin + dy * cos;

      boolean result =
          relativeX >= xMin && relativeX <= xMax && relativeY >= yMin && relativeY <= yMax;
      if (result) {
        callback.run();
      }
      return result;
    }
  }

  /**
   * Returns a singleton instance of FuelSim
   */
  public static FuelSim getInstance() {
      if (instance == null) {
          instance = new FuelSim();
      }

      return instance;
  }
}
