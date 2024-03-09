// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
    // FRONT
    private final CANSparkMax frontLeftDrive = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax frontRightDrive = new CANSparkMax(1, MotorType.kBrushless);

   // BACK
    private final CANSparkMax rearLeftDrive = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rearRightDrive = new CANSparkMax(3, MotorType.kBrushless);

   // CONTROLLERS (Operator / Driver)
    private final Joystick joystick = new Joystick(4);
    private final XboxController userDriver = new XboxController(5);
  // UPPER-MECHANISM
    private final CANSparkMax inOutSysMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax hook = new CANSparkMax(7, MotorType.kBrushless);

  // Drive 
    private MecanumDrive drive = new MecanumDrive(frontLeftDrive, rearLeftDrive, frontRightDrive, rearRightDrive);

  
  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

      // INTAKE
        if (userDriver.getRightBumperPressed()) { // powers intake for ring
            inOutSysMotor.set(.6); // prone to change according to constants || For Reakab: Make sure to adjust this intake to an according speed, just like a motor
        } else if (userDriver.getRightBumperReleased()) inOutSysMotor.set(0); // Sets intake to rest

        // HOOK
        if (userDriver.getLeftBumperPressed()) {
          hook.set(.3); // prone to change
        } else if (userDriver.getLeftBumperReleased()) inOutSysMotor.set(0);

      // JOYSTICK INPUTS
        double xSpeed = joystick.getRawAxis(1); // reads the input from the x-axis of the joystick | side-to-side motion or strafing
        double ySpeed = joystick.getRawAxis(0); // reads the input from the y-axis of the joystick | forward-backward motion
        double zRotation = joystick.getRawAxis(4); // reads the input from another axis of the joystick | rotation or turning
        drive.driveCartesian(xSpeed, ySpeed, zRotation); // x - axis speed, y - axis speed, z-rotation speed

      // Set steering angles for each wheel module using joystick axes
      /*
       *     Axis 2 controls the steering angles for the left side wheels.
             Axis 3 controls the steering angles for the right side wheels.
       */
        setSteeringAngle(frontLeftDrive, joystick.getRawAxis(2) * 90.0);  // Assuming axis 2 controls steering angle (multiply by 90 for full range)
        setSteeringAngle(frontRightDrive, joystick.getRawAxis(3) * 90.0); // Assuming axis 3 controls steering angle (multiply by 90 for full range)
        setSteeringAngle(rearLeftDrive, joystick.getRawAxis(2) * 90.0);    // Assuming axis 2 controls steering angle (multiply by 90 for full range)
        setSteeringAngle(rearRightDrive, joystick.getRawAxis(3) * 90.0);   // Assuming axis 3 controls steering angle (multiply by 90 for full range)

  }
  
  // method used to set steering angle for the swerve modules
  private void setSteeringAngle(CANSparkMax rearLeftDrive2, double angleInput) {
    // Convert joystick input to angle (assuming -1 to 1 input range)
    double angle = angleInput * 90.0; // Adjust as needed based on joystick configuration

    // Calculate speed control for the drive motor based on angle
    double speed = Math.cos(Math.toRadians(angle));

    // Set the speed for the drive motor
    rearLeftDrive2.set(speed);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
