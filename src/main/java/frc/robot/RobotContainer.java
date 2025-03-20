package frc.robot;

import static edu.wpi.first.units.Units.*;



import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.TriggerEvent;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


//import frc.robot.commands.ClimberComm;
import frc.robot.commands.Collect;
import frc.robot.commands.EleAu;
import frc.robot.commands.ElevatorA;
import frc.robot.commands.Ensamble;

import frc.robot.commands.Throw;
import frc.robot.commands.Thrownivel1;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algaes;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.climber;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  // private SendableChooser<AutoBuilder> mAutoChooser = new SendableChooser<>(); 

private Algaes mAlgaes = Algaes.getInstance();
private Intake mIntake = Intake.getInstance();
private final Ensamble ensamble = new Ensamble();

private ElevatorSub mElevatorSub = ElevatorSub.getInstance();
private final Collect mCollect = new Collect(mIntake, mElevatorSub);

private final ElevatorA mElevatorAH = new ElevatorA(-17.5, mElevatorSub);
private final ElevatorA mElevatorAM = new ElevatorA(-5, mElevatorSub);
private final Throw cThrow = new Throw(mIntake,mElevatorSub);
private final SendableChooser<Command> autoChooser;
//private final ClimberComm  climberComm = new ClimberComm();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
        SteerRequestType.MotionMagicExpo); 
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();




    public RobotContainer() {
        
NamedCommands.registerCommand("throw1", cThrow);
NamedCommands.registerCommand("collect", mCollect);
NamedCommands.registerCommand("ElevatorHigh", mElevatorAH);
NamedCommands.registerCommand("ElevatorMedio",mElevatorAM );

autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Autonomous",autoChooser);
configureBindings();

       mIntake.setDefaultCommand(ensamble);
        mElevatorSub.setDefaultCommand(ensamble);
        mAlgaes.setDefaultCommand(ensamble);
       // mClimber.setDefaultCommand(climberComm);
    }
private void configureBindings() {

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left) -
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left) - 
            )
        );

    //joystick.povUp().whileTrue(drivetrain.applyRequest(
       //     () -> robotDrive.withVelocityX(0.04* MaxSpeed).withVelocityY(0)));
   // joystick.povDown()
    //    .whileTrue(drivetrain.applyRequest(
       //     () -> robotDrive.withVelocityX(-0.04 * MaxSpeed).withVelocityY(0)));
    joystick.povLeft()
        .whileTrue(drivetrain.applyRequest(
            () -> robotDrive.withVelocityY(0.5).withVelocityX(0)));
    joystick.povRight()
      .whileTrue(drivetrain.applyRequest(
          () -> robotDrive.withVelocityY(-0.5).withVelocityX(0)));
   //joystick.leftTrigger().or(joystick.rightTrigger()).onTrue(()->robotDrive.withVelocityY(joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis()).withVelocityX(0));
  //drivetrain.applyRequest(()->robotDrive.withVelocityY(joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis()).withVelocityX(0));

       //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), joystick.getLeftX())) //-
        ));  

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
      joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 
       
       
    
         
           // reset the field-centric heading on left bumper press
          joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
       
    }
   

    public Command getAutonomousCommand() {
return autoChooser.getSelected();

}}
