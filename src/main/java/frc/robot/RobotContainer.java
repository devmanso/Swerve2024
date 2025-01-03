// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    // dualshock 4 controller for simulation stuff
    private final CommandPS4Controller ds4 = new CommandPS4Controller(0);

    private final CommandXboxController xbox = new CommandXboxController(1);
    private final Joystick bigJoystick = new Joystick(2);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("My New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    double aim() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    double range() {
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double deadband = 0.2;
                
                // Big joystick controls
                // double velocityX = -bigJoystick.getY();
                // double velocityY = -bigJoystick.getX();
                // double rotationalRate = (bigJoystick.getZ() * .5) * -1;
                double velocityX = -xbox.getLeftY();
                double velocityY = -xbox.getLeftX();
                double rotationalRate = -xbox.getRightX();
    
                // Apply deadband to velocityX
                if (Math.abs(velocityX) < deadband) {
                    velocityX = 0.0;
                } else {
                    velocityX = (velocityX - Math.signum(velocityX) * deadband) / (1 - deadband);
                }
    
                // Apply deadband to velocityY
                if (Math.abs(velocityY) < deadband) {
                    velocityY = 0.0;
                } else {
                    velocityY = (velocityY - Math.signum(velocityY) * deadband) / (1 - deadband);
                }
    
                // Apply deadband to rotationalRate
                if (Math.abs(rotationalRate) < deadband) {
                    rotationalRate = 0.0;
                } else {
                    rotationalRate = (rotationalRate - Math.signum(rotationalRate) * deadband) / (1 - deadband);
                }

                // TODO: Identify which button you need to press and change button number accordingly
                // if(bigJoystick.getRawButton(1)) {
                //     final double rotation = aim();
                //     rotationalRate = rotation;

                //     final double forward = range();
                //     velocityX = forward;

                //     // Limelight docs say to disable field-oriented drive, but idk how
                // }
    
                return drive
                    .withVelocityX(velocityX * MaxSpeed)
                    .withVelocityY(velocityY * MaxSpeed)
                    .withRotationalRate(rotationalRate * MaxAngularRate);
            })
        );

        xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        xbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))
        ));

        xbox.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        xbox.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        ds4.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        ds4.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        
        // This function needs testing
        xbox.leftBumper().whileTrue(drivetrain.applyRequest(() -> {
            final double rotation = -aim() * .5;
            final double forward = -range() * .5;
            return forwardStraight.withVelocityX(forward)
                .withVelocityY(0)
                .withRotationalRate(rotation * MaxAngularRate);
        }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xbox.back().and(xbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        xbox.back().and(xbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        xbox.start().and(xbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        xbox.start().and(xbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }
}
