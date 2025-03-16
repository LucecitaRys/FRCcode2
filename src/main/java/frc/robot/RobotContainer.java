package frc.robot;

import static edu.wpi.first.units.Units.*;



import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.climber;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
   //private SendableChooser<AutoBuilder> mAutoChooser = new SendableChooser<>(); 


private Intake mIntake = Intake.getInstance();
private final Ensamble ensamble = new Ensamble();
private final SendableChooser<Command> autoChooser;
private ElevatorSub mElevatorSub = ElevatorSub.getInstance();
private final Collect mCollect = new Collect(mIntake, mElevatorSub);

private final ElevatorA mElevatorAH = new ElevatorA(-17.5, mElevatorSub);
private final ElevatorA mElevatorAM = new ElevatorA(-5, mElevatorSub);
private final Throw cThrow = new Throw(mIntake,mElevatorSub);
//private final ClimberComm  climberComm = new ClimberComm();
private final climber mClimber = climber.getInstance();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
//private final PS4Controller joystick = new PS4Controller(0);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

   //UsbCamera usbCamera = new UsbCamera("Usb Camera", 1);



    public RobotContainer() {
        
NamedCommands.registerCommand("throw1", cThrow);
NamedCommands.registerCommand("collect", mCollect);
NamedCommands.registerCommand("ElevatorHigh", mElevatorAH);
NamedCommands.registerCommand("ElevatorMedio",mElevatorAM );
new EventTrigger("Elevator3").whileTrue(mElevatorAH);
//new EventTrigger("Elevator0").whileTrue(mElevatorAM);
autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Autonomous",autoChooser);
configureBindings();
//CameraServer.startAutomaticCapture();
//usbCamera.setResolution(320, 320);
//usbCamera.getVideoMode();
        
        mIntake.setDefaultCommand(ensamble);
        mElevatorSub.setDefaultCommand(ensamble);
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
        

       /*  if (joystick.getCircleButtonReleased()){
            drivetrain.applyRequest(() -> brake);
        }
        if (joystick.getCircleButtonReleased()){
            drivetrain.applyRequest(() ->
           point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), joystick.getLeftX())) //-
       );
        }
        
        if (joystick.getCircleButtonReleased()){
            drivetrain.applyRequest(() -> brake);
        }*/
        //joystick.getCircleButton().whileTrue(drivetrain.applyRequest(()->brake));
      /*  joystick.circle().whileTrue(drivetrain.applyRequest(() -> brake));
       joystick.cross().whileTrue(drivetrain.applyRequest(() ->
           point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), joystick.getLeftX())) //-
       ));
joystick.circle(drivetrain.applyRequest(() -> brake));
*/
       joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
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
         

           // Note that each routine should be run exactly once in a single log.
         //joystick.share().and(joystick.triangule()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         // joystick.share().and(joystick.scuare()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
         // joystick.options().and(joystick.triangule()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
          // joystick.options().and(joystick.scuare()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
   
           // reset the field-centric heading on left bumper press
          joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
//if(joystick.getCrossButtonReleased()){
//    drivetrain.seedFieldCentric();
//}
//joystick.getCrossButton().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

// if( joystick.getCrossButton()){
  //drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
 // }
  /*if( joystick.getCrossButton()){
  drivetrain.seedFieldCentric();
    }
    
 if( joystick.getShareButton() && joystick.getTriangleButton()){
    drivetrain.sysIdDynamic(Direction.kForward);
 }
 if( joystick.getShareButton() && joystick.getSquareButton()){
    drivetrain.sysIdDynamic(Direction.kReverse);
 }
 if( joystick.getOptionsButton() && joystick.getTriangleButton()){
    drivetrain.sysIdQuasistatic(Direction.kForward);
 }
 if( joystick.getOptionsButton() && joystick.getSquareButton()){
    drivetrain.sysIdQuasistatic(Direction.kReverse);
 }
 */
        drivetrain.registerTelemetry(logger::telemeterize);
        
    }

    public Command getAutonomousCommand() {
return autoChooser.getSelected();
//return new Autonomo();


//AutoBuilder.followPath("LOL xd");
/*drivetrain.applyRequest(() ->
drive.withVelocityX(0) // Drive forward with negative Y (forward)
    .withVelocityY(0) // Drive left with negative X (left) -
    .withRotationalRate(0) // Drive counterclockwise with negative X (left) - 
);
*/
}}
