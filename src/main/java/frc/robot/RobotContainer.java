package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter.HoodSub;
import frc.robot.subsystems.Shooter.ShooterSub;
import frc.robot.subsystems.hopper.IndexerSub;
import frc.robot.subsystems.hopper.IntakeSub;
import frc.robot.subsystems.hopper.SerializerSub;
import frc.robot.subsystems.hopper.SlapdownSub;
import frc.robot.subsystems.hopper.SlapdownSub.slapdownStates;
import frc.robot.autos.manualShot;
import frc.robot.commands.AutoAim;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.test.ClimbManual;
import frc.robot.commands.test.Flywheel;
import frc.robot.commands.test.HoodCMD;
import frc.robot.commands.test.IntakeManual;
import frc.robot.commands.test.SerializeTest;
import frc.robot.commands.test.SlapdownManual;
import frc.robot.commands.test.SlapdownStates;
import frc.robot.subsystems.*;

public class RobotContainer {
  /* Driver */
  private final Joystick driver = new Joystick(0);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton autoAimButton =
      new JoystickButton(driver, XboxController.Button.kA.value);

  /* Operator */
    // Operator
    private final Joystick operator = new Joystick(1);

    private final JoystickButton startIndexButton =
      new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

    private final JoystickButton startShootButton =
      new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    /*private final JoystickButton serializeTest = 
      new JoystickButton(operator, XboxController.Button.kB.value);*/

    private final JoystickButton startSerializeButton =
      new JoystickButton(operator, XboxController.Button.kB.value);

    /*private final JoystickButton flywheelTest =
      new JoystickButton(operator, XboxController.Button.kA.value);*/

    private final JoystickButton    intakeButton =
      new JoystickButton(operator, XboxController.Button.kA.value);

    /*private final JoystickButton hoodDownTest =
      new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton hoodUpTest = 
      new JoystickButton(operator, XboxController.Button.kY.value);*/

    private final JoystickButton climbUpTest =
      new JoystickButton(operator, XboxController.Button.kY.value);

    private final JoystickButton climbDownTest =
      new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton slapdownDown =
      new JoystickButton(operator, XboxController.Button.kStart.value);

    private final JoystickButton slapdownUp =
      new JoystickButton(operator, XboxController.Button.kBack.value);

    /*private final JoystickButton slapDownShovel = 
      new JoystickButton(operator, XboxController.Button.kBack.value);*/

  private final Joystick sysIdJoystick = new Joystick(2);
  private final JoystickButton quasiForward = new JoystickButton(sysIdJoystick, XboxController.Button.kA.value);
  private final JoystickButton quasiBackward = new JoystickButton(sysIdJoystick, XboxController.Button.kX.value);
  private final JoystickButton dynamicForward = new JoystickButton(sysIdJoystick, XboxController.Button.kB.value);
  private final JoystickButton dynamicBackward = new JoystickButton(sysIdJoystick, XboxController.Button.kY.value);


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final PoseEstimator poseEstimator = new PoseEstimator(s_Swerve);
  private final SlapdownSub m_slapdown = new SlapdownSub();
  private final SerializerSub m_serializer = new SerializerSub();
  private final ShooterSub m_flywheel = new ShooterSub();
  private final IndexerSub m_index = new IndexerSub();
  private final HoodSub m_hood = new HoodSub();
  private final IntakeSub m_intake = new IntakeSub();
  private final Climb m_climb = new Climb();


  /* Pose Estimator */
  /*private final Vision m_vision =
      new Vision(
          (pose, timestamp, stdDevs) ->
              poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs),
          "limelight");*/
          
  private final Vision m_vision = new Vision(
    new Vision.VisionConsumer() {
        @Override
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
            poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        }
    },
    "limelight"
    
);

  public PoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  private final HopperStructure hopperStructure = new HopperStructure(m_index, m_intake);
  private final ShooterStructure shooterStructure =
      new ShooterStructure(m_flywheel, m_hood, m_vision, m_serializer);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()
            ));

    NamedCommands.registerCommand("requestVisionShot", Commands.runOnce(() -> {shooterStructure.requestVisionShot(); }, shooterStructure));
    NamedCommands.registerCommand("setShooterIdle", Commands.runOnce(() -> {shooterStructure.requestIDLE(); }, shooterStructure));
    NamedCommands.registerCommand("shotFromChute", new manualShot(m_flywheel, m_hood));
    NamedCommands.registerCommand("runSerializer", new SerializeTest(m_serializer, 0.7));
    NamedCommands.registerCommand("requestIndexing", Commands.runOnce(() -> {hopperStructure.requestINDEXING(); }, hopperStructure));
    NamedCommands.registerCommand("setIndexerIdle", Commands.runOnce(() -> {hopperStructure.requestIDLE(); }, hopperStructure));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
          private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

     /*startIndexButton
        .whileTrue(Commands.run(hopperStructure::requestINDEXING, hopperStructure))
        .whileFalse(Commands.run(hopperStructure::stopIndexing, hopperStructure));*/

    /*startShootButton
        .whileTrue(Commands.run(shooterStructure::requestVisionShot, shooterStructure))
        .whileFalse(Commands.run(shooterStructure::stopShooting, shooterStructure));*/

    startIndexButton.whileTrue(
      Commands.runOnce(hopperStructure::requestINDEXING)
      ).whileFalse(
      Commands.runOnce(hopperStructure::requestIDLE)
      );

    startShootButton.onTrue(
      Commands.runOnce(() -> {
        if (shooterStructure.isReady()) {
            shooterStructure.requestIDLE();
        } else {
            shooterStructure.requestVisionShot();
        }
    }, shooterStructure)
  );

    /*slapDownShovel.onTrue(
      Commands.runOnce(() -> {
        if (m_slapdown.slapdownUP()) {
            m_slapdown.requestDown();
        } else {
            m_slapdown.requestUP();
        }
    }, m_slapdown)
    );*/

    //startSerializeButton.whileTrue(Commands.run(shooterStructure::shoot, shooterStructure));

    startSerializeButton.whileTrue(
      Commands.runOnce(shooterStructure::shoot)
      ).whileFalse(
      Commands.runOnce(shooterStructure::stopShooting)
      );

    //serializeTest.whileTrue(new SerializeTest(m_serializer, .6));

    /*flywheelQuasiFoward.whileTrue(m_flywheel.sysIdQuasi(SysIdRoutine.Direction.kForward));
    flywheelQuasiBackward.whileTrue(m_flywheel.sysIdQuasi(SysIdRoutine.Direction.kReverse));
    flywheelDynaForward.whileTrue(m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    flywheelDynaBackward.whileTrue(m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));*/
    // serializeTest.whileTrue(new SerializeTest(m_serializer, .7));

    intakeButton.whileTrue(new IntakeManual(m_intake, -0.9));

    climbUpTest.whileTrue(new ClimbManual(m_climb, 0.3));
    climbDownTest.whileTrue(new ClimbManual(m_climb, -0.3));

    slapdownUp.whileTrue(new SlapdownManual(m_slapdown, 0.5));
    slapdownDown.whileTrue(new SlapdownManual(m_slapdown, -0.5));

    autoAimButton.whileTrue(new AutoAim(s_Swerve, () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            m_vision
            ));

    /*hoodUpTest.onTrue(new HoodCMD(m_hood, 25));

    hoodDownTest.onTrue(new HoodCMD(m_hood, 0));    */

    quasiForward.whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    quasiBackward.whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    dynamicForward.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    dynamicBackward.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new exampleAuto(s_Swerve);
    return autoChooser.getSelected();
  }
}
