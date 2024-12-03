package frc.robot.commands.SwerveDrive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import lib.BlueShift.utils.JoystickUtils;

public class DriveSwerve extends Command {
  SwerveDrive swerveDrive;

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond));
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration.in(MetersPerSecondPerSecond));
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)));

  TrapezoidProfile.Constraints headingConstraints = new TrapezoidProfile.Constraints(
    Constants.SwerveDrive.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond) / 4,
    Constants.SwerveDrive.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)) / 4
  );
  ProfiledPIDController headingController = new ProfiledPIDController(0.05, 0, 0, headingConstraints);
  
  double headingSetpoint = 0;
  boolean setHeading = false;

  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> rotSpeed;
  Supplier<Boolean> fieldRelative;
  Supplier<Boolean> trackingSpeaker;
  
  public DriveSwerve(
    SwerveDrive swerveDrive,
    Supplier<Double> xSpeed,
    Supplier<Double> ySpeed,
    Supplier<Double> rotSpeed,
    Supplier<Boolean> fieldRelative,
    Supplier<Boolean> trackingSpeaker
  ) {
    this.swerveDrive = swerveDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldRelative = fieldRelative;
    this.trackingSpeaker = trackingSpeaker;

    headingController.enableContinuousInput(0, 2 * Math.PI);
    headingController.setTolerance(Constants.SwerveDrive.kHeadingTolerance.in(Radians));
    SmartDashboard.putData("HeadingController", headingController);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();
    double rot = rotSpeed.get();

    x = JoystickUtils.applyDeadbband(x, Constants.SwerveDrive.kJoystickDeadband);
    y = JoystickUtils.applyDeadbband(y, Constants.SwerveDrive.kJoystickDeadband);
    rot = JoystickUtils.applyDeadbband(rot, Constants.SwerveDrive.kJoystickDeadband);

    // Not rotating
    //if (rot == 0) {
    //  if (!setHeading) {
    //    headingSetpoint = swerveDrive.getHeading().getRadians() % (2 * Math.PI);
    //    setHeading = true;
    //  }
//
    //  if (setHeading && (x != 0 || y != 0)) rot = headingController.calculate(swerveDrive.getHeading().getRadians(), headingSetpoint);
    //} else {
    //  setHeading = false;
    //}

    //SmartDashboard.putBoolean("setheading", setHeading);
    //SmartDashboard.putNumber("headingsetpoint", headingSetpoint);
    
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    rot = rotLimiter.calculate(rot);

    x *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    y *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    rot *= Constants.SwerveDrive.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond);
    
    if (this.fieldRelative.get()) swerveDrive.driveFieldRelative(x, y, rot);
    else swerveDrive.driveRobotRelative(x, y, rot);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
