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
import frc.team670.robot.commands.MustangScheduler;

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
       // RobotContainer rc = new RobotContainer();
        MustangScheduler ms = MustangScheduler.getInstance();
        boolean caughtCS = false;

        TestSubsystem testSub1 = new TestSubsystem(1); // g
        TestSubsystem testSub2 = new TestSubsystem(2); // y
        TestSubsystem testSub3 = new TestSubsystem(3); // r
        TestSubsystem testSub4 = new TestSubsystem(1); // g
        TestSubsystem testSub5 = new TestSubsystem(1); // g
        TestSubsystem testSub6 = new TestSubsystem(1); // g
        
        TestCommand tc1 = new TestCommand(testSub1, testSub2, testSub3); 
        TestCommand tc2 = new TestCommand(testSub4, testSub5, testSub6);
        
        ms.schedule(tc1); //this one should fail
        ms.schedule(tc2); //this one should pass
        try{
          CommandScheduler.getInstance().schedule(tc2);
        }catch(RuntimeException e){
          caughtCS = true;
        }
        
        ms.run();

        //assertTrue(tc1.isEnded()); // using cancel instead
        assertTrue(tc1.isCancelled());
        assertFalse(tc1.isScheduled());
        assertTrue(caughtCS);
        assertTrue(tc2.isScheduled());
        //assert(ms.getCurrentlyScheduled().equals(tc2));
    }


  }