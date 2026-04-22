package frc.robot.commands;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.DriveIO;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Loads a PathPlanner {@code .auto} file and reconstructs its full command tree,
 * substituting every path-following segment with a
 * {@link PositionParameterizedPathCommand}.  All named commands, wait commands,
 * and group structures (sequential / parallel / race / deadline) are preserved
 * exactly as authored in the PathPlanner GUI.
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * // Basic — no pose reset
 * Command auto = ParameterizedAutoBuilder.buildAuto("TwoNote", drive);
 *
 * // With pose reset to the first path's starting pose
 * Command auto = ParameterizedAutoBuilder.buildAuto("TwoNote", drive, true);
 * }</pre>
 *
 * <h3>Supported node types</h3>
 * <ul>
 *   <li>{@code sequential}  → {@link SequentialCommandGroup}</li>
 *   <li>{@code parallel}    → {@link ParallelCommandGroup}</li>
 *   <li>{@code race}        → {@link ParallelRaceGroup}</li>
 *   <li>{@code deadline}    → {@link ParallelDeadlineGroup} (first child is deadline)</li>
 *   <li>{@code path}        → {@link PositionParameterizedPathCommand}</li>
 *   <li>{@code named}       → {@link NamedCommands#getCommand}</li>
 *   <li>{@code wait}        → {@link Commands#waitSeconds}</li>
 * </ul>
 */
public final class ParameterizedAutoBuilder {

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private ParameterizedAutoBuilder() {}

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Builds the auto without resetting the robot pose.
     *
     * @param autoName name of the {@code .auto} file (without extension)
     * @param drive    drive IO layer passed to each path command
     * @return the reconstructed auto command
     */
    public static Command buildAuto(String autoName, DriveIO drive) {
        return buildAuto(autoName, drive, false);
    }

    /**
     * Builds the auto, optionally prepending a pose-reset command that seeds
     * the odometry to the starting pose of the first path in the auto.
     *
     * @param autoName       name of the {@code .auto} file (without extension)
     * @param drive          drive IO layer passed to each path command
     * @param resetInitialPose if {@code true}, odometry is reset before the auto runs
     * @return the reconstructed auto command
     */
    public static Command buildAuto(String autoName, DriveIO drive, boolean resetInitialPose) {
        JsonNode root = loadAutoJson(autoName);
        Command autoCommand = buildCommand(root.path("command"), drive);

        if (!resetInitialPose) return autoCommand;

        // Find the starting pose from the first path referenced anywhere in the tree
        Pose2d startPose = findFirstPathStartPose(root.path("command"));
        if (startPose == null) return autoCommand;

        Command resetCmd = Commands.runOnce(() -> drive.setCurrentPose(startPose));
        return new SequentialCommandGroup(resetCmd, autoCommand);
    }

    // ── JSON loading ──────────────────────────────────────────────────────────

    private static JsonNode loadAutoJson(String autoName) {
        Path filePath = Filesystem.getDeployDirectory().toPath()
                .resolve("pathplanner/autos/" + autoName + ".auto");
        try {
            return MAPPER.readTree(Files.readString(filePath));
        } catch (IOException e) {
            throw new RuntimeException(
                    "[ParameterizedAutoBuilder] Could not load auto file: " + filePath, e);
        }
    }

    // ── Recursive command tree builder ────────────────────────────────────────

    private static Command buildCommand(JsonNode node, DriveIO drive) {
        if (node == null || node.isMissingNode() || node.isNull()) return Commands.none();

        String   type = node.path("type").asText("none");
        JsonNode data = node.path("data");

        return switch (type) {
            case "sequential" -> buildSequential(data, drive);
            case "parallel"   -> buildParallel(data, drive);
            case "race"       -> buildRace(data, drive);
            case "deadline"   -> buildDeadline(data, drive);
            case "path"       -> buildPath(data, drive);
            case "named"      -> buildNamed(data);
            case "wait"       -> buildWait(data);
            default -> {
                System.err.printf(
                    "[ParameterizedAutoBuilder] Unknown command type '%s' — substituting none()%n", type);
                yield Commands.none();
            }
        };
    }

    // ── Group builders ────────────────────────────────────────────────────────

    private static Command buildSequential(JsonNode data, DriveIO drive) {
        List<Command> cmds = parseChildren(data, drive);
        return new SequentialCommandGroup(cmds.toArray(Command[]::new));
    }

    private static Command buildParallel(JsonNode data, DriveIO drive) {
        List<Command> cmds = parseChildren(data, drive);
        return new ParallelCommandGroup(cmds.toArray(Command[]::new));
    }

    private static Command buildRace(JsonNode data, DriveIO drive) {
        List<Command> cmds = parseChildren(data, drive);
        return new ParallelRaceGroup(cmds.toArray(Command[]::new));
    }

    private static Command buildDeadline(JsonNode data, DriveIO drive) {
        List<Command> cmds = parseChildren(data, drive);
        if (cmds.isEmpty()) return Commands.none();

        // PathPlanner convention: the first child is the deadline command
        Command deadline = cmds.remove(0);
        return new ParallelDeadlineGroup(deadline, cmds.toArray(Command[]::new));
    }

    // ── Leaf builders ─────────────────────────────────────────────────────────

    private static Command buildPath(JsonNode data, DriveIO drive) {
        String pathName = data.path("pathName").asText();
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            System.err.printf(
                "[ParameterizedAutoBuilder] Could not load path '%s': %s%n", pathName, e.getMessage());
            return Commands.none();
        }
        return new PositionParameterizedPathCommand(drive, path);
    }

    private static Command buildNamed(JsonNode data) {
        String name = data.path("name").asText();
        Command cmd = NamedCommands.getCommand(name);
        if (cmd == null) {
            System.err.printf(
                "[ParameterizedAutoBuilder] Named command '%s' not registered — substituting none()%n", name);
            return Commands.none();
        }
        return cmd;
    }

    private static Command buildWait(JsonNode data) {
        double waitTime = data.path("waitTime").asDouble(0.0);
        return Commands.waitSeconds(waitTime);
    }

    // ── Shared helpers ────────────────────────────────────────────────────────

    private static List<Command> parseChildren(JsonNode data, DriveIO drive) {
        List<Command> cmds = new ArrayList<>();
        JsonNode children = data.path("commands");
        if (children.isArray())
            for (JsonNode child : children)
                cmds.add(buildCommand(child, drive));
        return cmds;
    }

    /**
     * Depth-first search for the first {@code "path"} node in the tree,
     * then loads that path and returns its starting pose.
     * Returns {@code null} if no path node is found.
     */
    private static Pose2d findFirstPathStartPose(JsonNode node) {
        if (node == null || node.isMissingNode() || node.isNull()) return null;

        String type = node.path("type").asText("none");

        if ("path".equals(type)) {
            String pathName = node.path("data").path("pathName").asText();
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                // getPreviewStartingHolonomicPose() is available in PathPlanner ≥ 2024.2;
                // fall back to the first point if it returns empty.
                return path.getStartingHolonomicPose().orElse(null);
            } catch (Exception e) {
                return null;
            }
        }

        // Recurse into children
        JsonNode children = node.path("data").path("commands");
        if (children.isArray())
            for (JsonNode child : children) {
                Pose2d result = findFirstPathStartPose(child);
                if (result != null) return result;
            }

        return null;
    }
}