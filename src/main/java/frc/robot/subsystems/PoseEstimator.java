// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.BaseStatusSignal;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Telemetry;
// import frc.robot.generated.TunerConstants;

// public class PoseEstimator extends SubsystemBase {

//   private final CommandSwerveDrivetrain swerveSubsystem;
//   private final Pigeon2Subsystem pigeon2Subsystem;
  
//   private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
//   private final Telemetry logger = new Telemetry(MaxSpeed);

//   // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
//   // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the others. 
//   // This in turn means the particualr component will have a stronger influence on the final pose estimate.
//   private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); //was 0.05, 0.05, deg to rad 5
//   private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9); //was 0.02, 0.02, 5
//   private static SwerveDrivePoseEstimator poseEstimator;
//   private static SwerveDrivePoseEstimator driftEstimator;
  
//   private final Field2d field2d = new Field2d();

//   private BaseStatusSignal[] signals;
//   //public int SuccessfulDaqs = 0;
//   //public int FailedDaqs = 0;

//   //private LinearFilter lowpass = LinearFilter.movingAverage(50);
//   //private double lastTime = 0;
//   //private double currentTime = 0;
//   //private double averageLoopTime = 0;
//   Rotation2d currAngle;
//   Rotation2d gyroRotation;


//   public PoseEstimator(CommandSwerveDrivetrain swerveSubsystem, Pigeon2Subsystem pigeon2Subsystem) {
//     this.swerveSubsystem = swerveSubsystem;
//     this.pigeon2Subsystem = pigeon2Subsystem;
//     currAngle = new Rotation2d(Math.PI/180 * pigeon2Subsystem.getPigeonYaw(true));
//     poseEstimator = new SwerveDrivePoseEstimator(
//       TunerConstants.KINEMATICS, 
//       pigeon2Subsystem.getGyroRotation(true), 
//       swerveSubsystem.getState().ModulePositions, 
//       new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 
//       stateStdDevs, 
//       visionMeasurementStdDevs);

//     // driftEstimator = new SwerveDrivePoseEstimator(
//     //   TunerConstants.KINEMATICS, 
//     //   pigeon2Subsystem.getGyroRotation(true), 
//     //   swerveSubsystem.getState().ModulePositions, 
//     //   new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 
//     //   stateStdDevs, 
//     //   visionMeasurementStdDevs);

//     SmartDashboard.putData("Field", field2d);


//     signals = new BaseStatusSignal[18];
//     BaseStatusSignal[] swerveSignals = swerveSubsystem.getSignals();
//     for(int i = 0; i<16; i++){
//       signals[i] = swerveSignals[i];
//     }
//     BaseStatusSignal[] pigeon2Signals = pigeon2Subsystem.getSignals();
//     signals[16] = pigeon2Signals[0];
//     signals[17] = pigeon2Signals[1];
//   }

//   @Override
//   public void periodic() {
//     gyroRotation = pigeon2Subsystem.getGyroRotation(true);
//     SmartDashboard.putNumber("Xpose", getPoseX());
//     SmartDashboard.putNumber("Ypose", getPoseY());
//     SmartDashboard.putNumber("RotPose", getPoseRotation().getDegrees());
//     SmartDashboard.putNumber("Pigeon rotation", gyroRotation.getDegrees());
//     SmartDashboard.putData(field2d);
//     double time = Timer.getFPGATimestamp();
//     SwerveModulePosition[] modulePoses = swerveSubsystem.getState().ModulePositions;
//     poseEstimator.updateWithTime(time, gyroRotation, modulePoses);
//     // driftEstimator.updateWithTime(time, gyroRotation, modulePoses);
//     field2d.setRobotPose(poseEstimator.getEstimatedPosition());
//   }

//   /**
//    * Get the current pose of the robot using the pose estimator
//    * @return Pose2d representing the current estimated X, Y, and Theta of the robot
//    */
//   public Pose2d getPose() {
//     return poseEstimator.getEstimatedPosition();
//   }

//   public ChassisSpeeds getChassisSpeeds() {
//     return TunerConstants.KINEMATICS.toChassisSpeeds(swerveSubsystem.getState().ModuleStates);
//   }

//    public static boolean onRed() {
//     var alliance = DriverStation.getAlliance();
//     if (alliance.isPresent()) {
//         return alliance.get() == DriverStation.Alliance.Red;
//     }
//     return false;
//   } 


//   /**
//    * Get the current X position of the robot using the pose estimator
//    * @return double representing the current estimated X of the robot
//    */
//   public double getPoseX() {
//     return poseEstimator.getEstimatedPosition().getX();
//   }
//   /**
//    * sets the current angle the robot will adjust to
//    * @param double representing the angle in degrees
//    * 
//    */
//   public void setCurrAngle(double degAngle)
//   {
//     currAngle = Rotation2d.fromDegrees(degAngle);
//   }

//   /**
//    * gets the curr angle
//    * @return currAngle (degrees)
//    * 
//    */
//   public Rotation2d getCurrAngle()
//   {
//     return currAngle;
//   }


//   /**
//    * adds the inputted radian value to currAngle
//    * @param the radians to add
//    */
//   public void addToCurrAngle(double radToAdd)
//   {
//     currAngle = currAngle.plus(Rotation2d.fromRadians(radToAdd));
//   }
//   /**
//    * Get the current Y position of the robot using the pose estimator
//    * @return double representing the current estimated Y of the robot
//    */
//   public double getPoseY() {
//     return poseEstimator.getEstimatedPosition().getY();
//   }

//   /**
//    * Get the current Theta position of the robot using the pose estimator
//    * @return double representing the current estimated Theta of the robot
//    */
//   public double getPoseTheta() {
//     return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
//   }

//   /**
//    * Get the current Theta position of the robot using the pose estimator
//    * @return Rotation2d representing the current estimated Theta of the robot
//    */
//   public Rotation2d getPoseRotation() {
//     return poseEstimator.getEstimatedPosition().getRotation();
//   }
//   // public Rotation2d getDriftRotation()
//   // {
//   //   return driftEstimator.getEstimatedPosition().getRotation();
//   // }

//   /**
//    * Sets the pose of the pose estimator
//    * @param pose to set the pose estimator to
//    */
//   public void setPose(Pose2d pose) {
//     poseEstimator.resetPosition(pigeon2Subsystem.getGyroRotation(true), swerveSubsystem.getState().ModulePositions, pose);
//   }
//   // public void setDriftPose(Pose2d pose)
//   // {
//   //   driftEstimator.resetPosition(pigeon2Subsystem.getGyroRotation(true), swerveSubsystem.getState().ModulePositions, pose);
//   // }

//   /**
//    * Draws a trajectory on the field2d object to view on shuffleboard
//    * @param trajectory to be drawn on the field2d object
//    */
//   public void setTrajectoryField2d(Trajectory trajectory) {
//     field2d.getObject("traj").setTrajectory(trajectory);
//   }

//   public void addVisionMeasurement(Pose2d pose, double resultTime) {
//     poseEstimator.addVisionMeasurement(pose, resultTime);
//   }

//   public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDev) {
//     poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
//   }

//   public Pose2d getEstimatedPosition() {
//     return poseEstimator.getEstimatedPosition();
//   }
// }