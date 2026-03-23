package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Indexing.FeedShooterFactory;
import frc.robot.commands.Intake.IntakeDeploy;
import frc.robot.commands.Intake.IntakeStow;
import frc.robot.commands.Intake.RunIntakeRollers;
import frc.robot.subsystems.Drive.DriveIO;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Intake.IntakeDeployIO;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

public class AllSystemsTest {
    private final DriveIO m_drive;
    private final IntakeDeployIO m_intakeDeploy;
    private final IntakeRollersTalonFX m_intakeRollers;
    private final Kicker m_kicker;
    private final PreIndexer m_preindexer;
    private final FlywheelIO m_flywheel;

    public AllSystemsTest(
        DriveIO drive,
        IntakeDeployIO intakeDeploy,
        IntakeRollersTalonFX intakeRollers,
        Kicker kicker,
        PreIndexer preindexer,
        FlywheelIO flywheel
    ) {
        m_drive = drive;
        m_intakeDeploy = intakeDeploy;
        m_intakeRollers = intakeRollers;
        m_kicker = kicker;
        m_preindexer = preindexer;
        m_flywheel = flywheel;
    }


    public SequentialCommandGroup getSystemsTest(){
        FeedShooterFactory indexers = new FeedShooterFactory(m_kicker, m_preindexer);
        SequentialCommandGroup test = new SequentialCommandGroup(
            new IntakeDeploy(m_intakeDeploy),
            new WaitCommand(2),
            new IntakeStow(m_intakeDeploy),
            new WaitCommand(2),
            new RunIntakeRollers(m_intakeRollers).withTimeout(1),
            new WaitCommand(1),
            new RunCommand(()->indexers.start()).withTimeout(1).finallyDo(()->indexers.stop()),
            new WaitCommand(1),
            new SetShooterToDefinedStateOnce(m_flywheel, ShooterConstants.kDefaultShooterState),
            new WaitCommand(1),
            new RunCommand(()->m_drive.driveRobotRelativeChassis(new ChassisSpeeds(0.5, 0, 0)), m_drive).withTimeout(0.5),
            new InstantCommand(m_drive::stop, m_drive)
        );
        return test;
    }
}
