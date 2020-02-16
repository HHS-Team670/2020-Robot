package frc.team670.robot.subsystems.climber;
 
import frc.team670.robot.subsystems.climber.Pull;
import frc.team670.robot.subsystems.MustangSubsystemBase;
 
/**
* @version Pallavi & Eugenia
*/
 
public class Climber extends MustangSubsystemBase{
 
   private Pull rightPull;
   private Pull leftPull;
   private boolean leftPullHookedOnBar;
   private boolean rightPullHookedOnBar;
 
   public Climber() {
       this.leftPull = new Pull(false);
       this.rightPull = new Pull(true);
       leftPullHookedOnBar = false;
       rightPullHookedOnBar = false;
   }
  
   public void set(double speed){
       leftPull.setPower(speed);
       rightPull.setPower(speed);  
   }
 
   public Pull getRightPull(){
       return rightPull;
   }
 
   public Pull getLeftPull(){
       return leftPull;
   }
 
   public void climb(double heightCM) {
        leftPull.climb(heightCM);
        rightPull.climb(heightCM);
   }
   
   public boolean hookOnBar() {
       leftPullHookedOnBar = leftPull.hookOnBar();
       rightPullHookedOnBar = rightPull.hookOnBar();
       return rightPullHookedOnBar && leftPullHookedOnBar;
   }
 
   public boolean isAtTarget() {
        return leftPull.isAtTarget() && rightPull.isAtTarget();
   }
 
   @Override
   public HealthState checkHealth() {
       // TODO Auto-generated method stub
       return null;
   }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }
 
}   
