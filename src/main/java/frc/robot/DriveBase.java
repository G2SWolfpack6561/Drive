package frc.robot;
import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveBase
{
   public SwerveDrive swerveDrive;
   public  SwerveController swerveController;
   double  maximumSpeed = Units.feetToMeters(15.1);

   private boolean       control_active = false;
   private Translation2d x_y_ctrl = new Translation2d( 0.0, 0.0 );

   //                                                 kP      Ki      Kd
   private PIDController x_y_PID = new PIDController( 6.0, 0.0, 0.0 );
   //
   //   Create a YAGSL swerve drive and PID controllers
   //
   //
   DriveBase( )
   {
      try
      {
         swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(maximumSpeed);
         swerveController = swerveDrive.swerveController;
         swerveController.thetaController.setTolerance( Math.PI/90.0, 0.1 );
         swerveController.thetaController.setPID(0.85, 0.0, 0.02);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      //swerveDrive.setHeadingCorrection( true );

      x_y_PID.setTolerance( 0.05, 0.05);
      x_y_PID.setIntegratorRange(-0.04, 0.04);
      swerveDrive.setMotorIdleMode( true );
   }
   //
   //   Must be called periodically to update telemetry
   //
   //
   void periodic( )
   {
      Pose2d current_pose = swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition( );
      SmartDashboard.putNumber("RobotX", current_pose.getX( ) );
      SmartDashboard.putNumber("RobotY", current_pose.getY( ) );
      SmartDashboard.putNumber("RobotZ", current_pose.getRotation( ).getDegrees( ) );
      SmartDashboard.putNumber("ctrlx", x_y_ctrl.getX());
      SmartDashboard.putNumber("ctrly", x_y_ctrl.getY());
   }
   //
   //   Add vision measurement collected to odometry
   //
   //
   void addVisionMeasurement( Pose2d new_pose, double timestamp )
   {
      swerveDrive.addVisionMeasurement( new_pose, timestamp );
   }
   //
   //   Drive using velocities m/s
   //
   //
   public void drive( double x, double y, double rot )
   {
      swerveDrive.driveFieldOriented( new ChassisSpeeds( x, y, rot ) );
   }

   public void driveRobot( double x, double y, double rot )
   {
      swerveDrive.drive( new ChassisSpeeds(x, y, rot) );
   }
   //
   //   Drive using raw joystick readings with second joystick 
   //
   //
   public void drive( double x, double y, double zx, double zy )
   {
      driveHeading( x, y, swerveController.getJoystickAngle( zx, zy ) );
   }
   //
   //   Drive translating on the field but always orient towards
   //   given Pose2d
   //
   //
   public boolean driveFacing( double x, double y, Pose2d anchor )
   {
      return driveFacing( x, y, anchor.getTranslation() );
   }
   //
   //   Drive translating on the field but always orient towards
   //   given Translation2d
   //
   //
   public boolean driveFacing( double x, double y, Translation2d anchor )
   {
      Pose2d pose = swerveDrive.getPose();
      double radians = swerveController.getJoystickAngle( pose.getY() - anchor.getY(), pose.getX() - anchor.getX() );
      System.out.println("Degrees: " + Math.toDegrees(radians));
      return driveHeading( x, y, radians );
   }
   //
   //   Drive translating on the field but always orient towards
   //   given Rotation2d
   //
   //
   public boolean driveFacing( double x, double y, Rotation2d anchor )
   {
      return driveHeading( x, y, anchor.getRadians() );
   }
   //
   //   Drive translating on the field but always orient towards
   //   given Rotation2d
   //
   //
   public boolean driveHeading( double x, double y, double radians )
   {
      Pose2d         pose     = swerveDrive.getPose();
      ChassisSpeeds newSpeeds = swerveController.getTargetSpeeds( x, y, radians, pose.getRotation().getRadians(), 3.0 );

      if ( Math.abs(swerveController.thetaController.getPositionError())<.07 )
      //swerveController.thetaController.atSetpoint() )
      {
         newSpeeds = new ChassisSpeeds( x, y, 0.0 ); 
      }
      swerveDrive.driveFieldOriented( newSpeeds );
      return ! swerveController.thetaController.atSetpoint();
   }
   //
   //   Move and orient to new_pose from current location
   //
   //
   public boolean move_Pose2d( Pose2d pose2d )
   {
      Pose2d  pose     = swerveDrive.getPose();
      double  x1       = pose.getX() - pose2d.getX();
      double  y1       = pose.getY() - pose2d.getY();
      double  x2       = 0.0;
      double  y2       = 0.0;
      double  ctrl;
      double  distance = Math.hypot(x1, y1);
      x_y_ctrl = new Translation2d(  x1, y1 );

      if ( ! control_active )
      {
         swerveController.lastAngleScalar = pose.getRotation( ).getRadians( );
         swerveController.thetaController.reset();
         x_y_PID.reset( );
         x_y_PID.setSetpoint( 0.0 );
      }
      ctrl           =  x_y_PID.calculate( distance );
      control_active = !x_y_PID.atSetpoint( ) || !swerveController.thetaController.atSetpoint( );
      System.out.println( "xy: " + x_y_PID.atSetpoint() + " rot: " + swerveController.thetaController.atSetpoint() );

      if ( ! x_y_PID.atSetpoint( ) )
      {
         x2 = x1 * ctrl / distance;
         y2 = y1 * ctrl / distance;
      }
      
      swerveDrive.driveFieldOriented(
             swerveController.getTargetSpeeds( x2, y2,
                                               pose2d.getRotation().getRadians(),
                                               pose.getRotation().getRadians(),
                                               0.5 ) );
      return control_active;
   }
   //
   //   Tells PID controllers to reset upon next start of control period
   //
   //
   public void resetControl()
   {
      control_active = false;
   }
   //
   //   Return the theoretical maximum velocity of this drive system
   //
   //
   public double getMaximumVelocity()
   {
      return swerveDrive.getMaximumChassisVelocity();
   }
   //
   //   Returns the latest pose of the robot from odometery
   //
   //
   public Pose2d getPose()
   {
      return swerveDrive.getPose();
   }

   public void stopDrive()
   {
      swerveDrive.lockPose();
   }

}

