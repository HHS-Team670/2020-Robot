package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import frc.team670.robot.RobotContainer;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.commands.MustangCommandBase;

import frc.team670.robot.commands.TestCommand;
import frc.team670.robot.commands.TestSubsystem;

  /**
  * Commented out part of the test for building. Tests for this have passed, but uncomment testMC() to add another.
  */
    public class CommandTest{
        
    /**
     *  This is a really simple example but checkCommandHealth()'s logic is the same no matter what.
     *  Simply replace these TestSubsystems and TestCommands with whatever Subsystems and Commands 
     *  you want to test health checks for. 
     */  

    @Test
    public void testMC(){
        RobotContainer rc = new RobotContainer();

        TestSubsystem test1 = new TestSubsystem(1);
        TestSubsystem test2 = new TestSubsystem(2);
        TestSubsystem test3 = new TestSubsystem(3);
        TestSubsystem test4 = new TestSubsystem(1);
        TestSubsystem test5 = new TestSubsystem(1);
        TestSubsystem test6 = new TestSubsystem(1);
        
      //  TestCommand tc1 = new TestCommand(test1, test2, test3); 
        TestCommand tc2 = new TestCommand(test4, test5, test6);
        
        CommandScheduler.getInstance().onCommandInitialize(command -> CommandTest.checkCommandsHealth(command));
        CommandScheduler.getInstance().onCommandExecute(command -> ((TestCommand)(command)).setRunning(true));

       // CommandScheduler.getInstance().schedule(tc1);
        CommandScheduler.getInstance().schedule(tc2);
        CommandScheduler.getInstance().run();
       
       // assertTrue(tc1.isEnded());
        assertTrue(tc2.isRunning());

    }


    // This is the same logic as what is run in Robot.java for each command required.
    public static void checkCommandsHealth(Command command){
        if (command instanceof MustangCommandBase){
          for (MustangSubsystemBase s: ((MustangCommandBase)command).getHealthRequirements().keySet()){
            if (s.getHealth(false).getId() > ((MustangCommandBase)command).getHealthRequirements().get(s).getId()){
                CommandScheduler.getInstance().cancel(command);
                Logger.consoleLog("Canceled command");
                Logger.consoleLog("Target state: " + ((MustangCommandBase)command).getHealthRequirements().get(s));
                Logger.consoleLog("Actual state: " + s.getHealth(true));
            }
          }
        }
    }

  }