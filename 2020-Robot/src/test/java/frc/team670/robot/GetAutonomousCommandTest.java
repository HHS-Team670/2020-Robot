package frc.team670.robot;

import org.junit.Test;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator2BallSide;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToTrench;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

import org.junit.Assert;
import static org.mockito.Mockito.*;

public class GetAutonomousCommandTest {


  @Test
  public void testGetAutonomousCommand() throws Exception{
    //MustangCommand autonomousCommand = RobotContainer.getAutonomousCommand();
    DriveBase driveBase = mock(DriveBase.class);
    Intake intake = mock(Intake.class);
    Conveyor conveyor = mock(Conveyor.class);
    Shooter shooter = mock(Shooter.class);
    Indexer indexer = mock(Indexer.class);
    Turret turret = mock(Turret.class);
    MustangCoprocessor coprocessor = mock(MustangCoprocessor.class);
    ShootFromBaseLineThenToTrench command = new ShootFromBaseLineThenToTrench(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);

  }

    
}