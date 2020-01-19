package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.junit.Test;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.commands.MustangCommandBase;

  /**
  * Commented out part of the test for building. Tests for this have passed, but uncomment testMC() to add another.
  */
    public class CommandTest{
        
    /**
     *  This is a really simple example but checkCommandHealth()'s logic is the same no matter what.
     *  Simply replace these TestSubsystems and TestCommands with whatever Subsystems and Commands 
     *  you want to test health checks for. 
     */  

    // @Test
    // public void testMC(){
    //     TestSubsystem test1 = new TestSubsystem(1);
    //     TestSubsystem test2 = new TestSubsystem(2);
    //     TestSubsystem test3 = new TestSubsystem(3);
        
    //     TestCommand tc = new TestCommand(test1, test2, test3);

    //     checkCommandHealth(tc);

    // }


    // This is the same logic as what is run in Robot.java for each command required.
    public static void checkCommandHealth(Command command){
        if (command instanceof MustangCommandBase){
          for (MustangSubsystemBase s: ((MustangCommandBase)command).getHealthRequirements().keySet()){
            if (s.getHealth(true).getId() > ((MustangCommandBase)command).getHealthRequirements().get(s).getId()){
                System.out.println("Canceled command");
                System.out.println("Target state: " + ((MustangCommandBase)command).getHealthRequirements().get(s));
                System.out.println("Actual state: " + s.getHealth(true));
            }
          }
        }
    }

  }