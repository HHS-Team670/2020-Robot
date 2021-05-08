package frc.team670.robot.commands;

import static org.mockito.Mockito.mock;

import java.security.Policy;

import org.junit.Test;

import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.utils.math.interpolable.LinearRegression;
import frc.team670.robot.utils.math.interpolable.PolynomialRegression;

public class ShooterTest {

    private static final double[] measuredDistancesMeters = {
        3.32232,  // 10.9 ft  2125 rpm 
        4.572, // 15 ft  2275 rpm 
        7.3152, // 24 ft 2575 rpm
        8.6868, 
        9.4488,// trench (28-29ft)
      };
    
      private static final double[] measuredRPMs = {
        2125,  // 10.9 ft  2125 rpm 
        2275, // 15 ft  2275 rpm 
        2575, // 24 ft 2575 rpm 
        2725, 
        3100
      };
    
    private static final PolynomialRegression speedAtDistance = new PolynomialRegression(measuredDistancesMeters, measuredRPMs, 4);
    

    @Test
    public void testGetAutonomousCommand() throws Exception{
    //MustangCommand autonomousCommand = RobotContainer.getAutonomousCommand();

    
    
        
        System.out.println( Math.max(Math.min(speedAtDistance.predict(7.31), 3100), 2100));
        System.out.println(speedAtDistance.toString());

  }
    
}
