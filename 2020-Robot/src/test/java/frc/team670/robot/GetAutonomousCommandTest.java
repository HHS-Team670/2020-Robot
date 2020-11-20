package frc.team670.robot;

import org.junit.Test;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator2BallSide;
import frc.team670.robot.commands.auton.baseline.ShootThenBack;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator3BallMid;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToTrench;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import static org.mockito.Mockito.*;

public class GetAutonomousCommandTest {


  @Test
  public void testGetAutonomousCommand() throws Exception{
    //MustangCommand autonomousCommand = RobotContainer.getAutonomousCommand();
    DriveBase drivebase = mock(DriveBase.class);
    Intake intake = mock(Intake.class);
    Conveyor conveyor = mock(Conveyor.class);
    Shooter shooter = mock(Shooter.class);
    Indexer indexer = mock(Indexer.class);
    Turret turret = mock(Turret.class);
    MustangCoprocessor processor = mock(MustangCoprocessor.class);

    ShootFromBaseLineThenToTrench shootFromBaseLineRightToTrenchCommand = new ShootFromBaseLineThenToTrench(StartPosition.RIGHT, drivebase, intake, conveyor, shooter, indexer, turret, processor);
    ShootFromBaseLineThenToTrench shootFromBaseLineLeftToTrenchCommand = new ShootFromBaseLineThenToTrench(StartPosition.LEFT, drivebase, intake, conveyor, shooter, indexer, turret, processor);
    ShootFromBaseLineThenToTrench shootFromBaseLineCenterToTrenchCommand = new ShootFromBaseLineThenToTrench(StartPosition.CENTER, drivebase, intake, conveyor, shooter, indexer, turret, processor);

    ShootFromBaseLineThenToGenerator2BallSide shootFromBaseLineRightToGenerator2Command = new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.RIGHT, drivebase, intake, conveyor, shooter, indexer, turret, processor);
    ShootFromBaseLineThenToGenerator2BallSide shootFromBaseLineLeftToGenerator2Command = new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.LEFT, drivebase, intake, conveyor, shooter, indexer, turret, processor);
    ShootFromBaseLineThenToGenerator2BallSide shootFromBaseLineCenterToGenerator2Command = new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.CENTER, drivebase, intake, conveyor, shooter, indexer, turret, processor);

    ShootFromBaseLineThenToGenerator3BallMid shootFromBaseLineToGeneratorRight3Command = new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.RIGHT, drivebase, intake, conveyor, shooter, indexer, turret, processor);    
    ShootFromBaseLineThenToGenerator3BallMid shootFromBaseLineToGeneratorLeft3Command = new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.LEFT, drivebase, intake, conveyor, shooter, indexer, turret, processor);
    ShootFromBaseLineThenToGenerator3BallMid shootFromBaseLineToGeneratorCenter3Command = new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.CENTER, drivebase, intake, conveyor, shooter, indexer, turret, processor);
    
    ShootThenBack shootThenBackCommand = new ShootThenBack(drivebase, intake, conveyor, shooter, indexer, turret, processor);
    
  }

    
}