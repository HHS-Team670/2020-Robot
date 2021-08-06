// package frc.team670.robot.commands.routines;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.robot.commands.indexer.SendAllBalls;
// import frc.team670.robot.commands.indexer.StageOneBallToShoot;
// import frc.team670.robot.commands.shooter.StartShooter;
// import frc.team670.robot.commands.shooter.StartShooterByDistance;
// import frc.team670.robot.subsystems.DriveBase;
// import frc.team670.robot.subsystems.Indexer;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.robot.subsystems.Shooter;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

// public class ShootAllBalls extends SequentialCommandGroup implements MustangCommand {

//     private Indexer indexer;
//     private Shooter shooter;
//     private Map<MustangSubsystemBase, HealthState> healthReqs;

//     public ShootAllBalls(Indexer indexer, Shooter shooter, DriveBase driveBase){
//         this.indexer = indexer;
//         this.shooter = shooter;
//         addRequirements(indexer, shooter);
//         healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
//         healthReqs.put(indexer, HealthState.GREEN);
//         healthReqs.put(shooter, HealthState.GREEN);

//         addCommands(
//             new ParallelCommandGroup(
//                 new StartShooterByDistance(shooter, driveBase),
//                 new StageOneBallToShoot(indexer)
//             ),
//             new SendAllBalls(indexer)
//         );
//     }

//     /**
//      * This sequence will only end if we tell it to, otherwise, we keep blasting
//      */
//     @Override
//     public void end(boolean interrupted) {
//         if (interrupted) {
//             shooter.stop();
//             indexer.stopUpdraw();
//         }
//     }

//     @Override
//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return this.healthReqs;
//     }
    
// }