/*package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FollowPath extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Trajectory trajectory;
    private final Rotation2d targetRotation;
   

    public FollowPath(Trajectory trajectory, Rotation2d targetRotation) {
        this.drivetrain = TunerConstants.createDrivetrain();
        this.trajectory = trajectory; 
        this.targetRotation = targetRotation;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
       drivetrain.setTrajectory(trajectory, targetRotation);
        
    }

    @Override
    public void execute() {
        // Aquí puedes agregar cualquier lógica adicional que necesites durante la ejecución
    }

    @Override
    public boolean isFinished() {
        return drivetrain.isTrajectoryFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drivetrain.stop();
        }
    }
}
*/