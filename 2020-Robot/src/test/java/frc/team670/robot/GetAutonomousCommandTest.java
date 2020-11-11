package frc.team670.robot;

import org.junit.Test;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator2BallSide;
import org.junit.Assert;
import static org.mockito.Mockito.*;

class GetAutonomousCommandTest {


  @Test
  public void testGetAutonomousCommand() {
    RobotContainer robotContainer = mock(RobotContainer.class);
    MustangCommand autonomousCommand = RobotContainer.getAutonomousCommand();

    Assert.assertEquals(autonomousCommand, mock(ShootFromBaseLineThenToGenerator2BallSide.class));

  }

    
}