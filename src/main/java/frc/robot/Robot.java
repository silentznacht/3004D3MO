// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import frc.robot.Drivetrain;


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
      private final CANSparkMax frontLeftModule = new CANSparkMax(0, MotorType.kBrushless);
      private final CANSparkMax frontRightModule = new CANSparkMax(1, MotorType.kBrushless);

    // BACK
      private final CANSparkMax rearLeftModule = new CANSparkMax(2, MotorType.kBrushless);
      private final CANSparkMax rearRightModule = new CANSparkMax(3, MotorType.kBrushless);

    // CONTROLLERS (Operator / Driver)
      private final Joystick joystick = new Joystick(4);
      private final XboxController userDriver = new XboxController(5);
    // UPPER-MECHANISM
      private final CANSparkMax inOutSysMotor = new CANSparkMax(6, MotorType.kBrushless);
      private final CANSparkMax hook = new CANSparkMax(7, MotorType.kBrushless);

    // Drive 
      private final Drivetrain m_swerve = new Drivetrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
      private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
      private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
      private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    
    // Gyro 
      private AHRS gyro;

      private void driveWithJoystick(boolean fieldRelative) {
            // Get the x speed. 
              final var xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getY(), 0.02)) * Drivetrain.kMaxSpeed;
            // Get the y speed or sideways/strafe speed
              final var ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getX(), 0.02)) * Drivetrain.kMaxSpeed;
            // Get the rate of angular rotation, We want a positive value when we pull to the left (CCW is positive in mathematics).
              final var rotate = -m_rotLimiter.calculate(MathUtil.applyDeadband(joystick.getX(), 0.02)) * Drivetrain.kMaxAngularSpeed;
            // Drive Train
              m_swerve.drive(xSpeed, ySpeed, rotate, fieldRelative, getPeriod());
      }

      private void ahrsGyroAutos (boolean gyroStart) {
          double yaw = (float) gyro.getYaw();
          double pitch = (float) gyro.getPitch();
          double roll = (float) gyro.getRoll();

            if (yaw == 0 && pitch == 0 & roll == 0) {
                
            }
      }

      @Override
      public void autonomousInit() {
        driveWithJoystick(false);
        m_swerve.updateOdometry();
        gyro = new AHRS(SPI.Port.kMXP); // Assuming NavX2 is connected via MXP port
        
      }
      @Override
      public void autonomousPeriodic() {
        driveWithJoystick(false);
        m_swerve.updateOdometry();
      }

      @Override
      public void teleopPeriodic() {

        driveWithJoystick(true);

        /*
          Dont mind the code commented out Below is kinda of a demo, I was an idiot and mixed our drive train with meccanum so until I figure out how to instantiate 
          the swerve drive train object in our code, this exmp i found on wpi docs will have to do. 
                        - Kevin E :3
        */



        /***********************************************************************************************************************************************/
        //                                                                  /\_/\                                                                      //
        //                                                                 ( o.o )                                                                     //
        //                                                                  > ^ <                                                                      //
        //                                                          [Author: @silentznacht]                                                            //
        /**********************************************************************************************************************************************/





          // INTAKE
            if (userDriver.getRightBumperPressed()) { // powers intake for ring
                inOutSysMotor.set(.6); // prone to change according to constants || For Reakab: Make sure to adjust this intake to an according speed, just like a motor
            } else if (userDriver.getRightBumperReleased()) inOutSysMotor.set(0); // Sets intake to rest

          // HOOK
            if (userDriver.getLeftBumperPressed()) {
              hook.set(.3); // prone to change
            } else if (userDriver.getLeftBumperReleased()) inOutSysMotor.set(0);

          // JOYSTICK INPUTS (In Progress?) 
            // double xSpeed = joystick.getRawAxis(0); // reads the input from the x-axis of the joystick | side-to-side motion or strafing
            // double ySpeed = joystick.getRawAxis(1); // reads the input from the y-axis of the joystick | forward-backward motion
            // double zRotation = joystick.getRawAxis(2); // reads the input from another axis of the joystick | rotation or turning
            
          // DRIVE

            //swerveDrivetrain.driveCartesian(xSpeed, ySpeed, zRotation); // x - axis speed, y - axis speed, z-rotation speed

          // Set steering angles for each wheel module using joystick axes
            /*
            *                    [NOTES TO REMEMBER]
            *     Axis 1 controls the steering angles for the left side wheels. 
                  Axis 2 controls the steering angles for the right side wheels.
                  Axis 1 and 2 are associated with the joystick's horizontal and vertical movements.
                  
                    Since swerve drive systems often require independent control of each wheel module's steering angle, 
                    it's common to use one analog stick for controlling the steering angles of the left side wheels and 
                    the other analog stick for controlling the steering angles of the right side wheels. This allows for 
                    intuitive control, similar to driving a car with a steering wheel

                  What does this do?
                    It detects anolog axis movement
                      - If axis 2 detected => 90 and if anologAxis != 2 || 3 => 0

            */
            // setSteeringAngle(frontLeftModule, joystick.getRawAxis(1) * 90.0);  // axis 2 controls steering angle (multiply by 90 for full range)
            // setSteeringAngle(frontRightModule, joystick.getRawAxis(2) * 90.0); // axis 3 controls steering angle (multiply by 90 for full range)
            // setSteeringAngle(rearLeftModule, joystick.getRawAxis(1) * 90.0);    // axis 2 controls steering angle (multiply by 90 for full range)
            // setSteeringAngle(rearRightModule, joystick.getRawAxis(2) * 90.0);   // axis 3 controls steering angle (multiply by 90 for full range)

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
