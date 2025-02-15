package frc.robot.subsystems;

import frc.lib.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AprilTagEstimator extends SubsystemBase {
    private Pose3d limeLightPose;
    private Pose2d limeLightPose2D;
    private String bestLimelight = "none";
    private final CommandSwerveDrivetrain drivetrain;
    
    // Main Limelight
    NetworkTable mainTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry mainTx = mainTable.getEntry("tx");
    NetworkTableEntry mainTy = mainTable.getEntry("ty");
    NetworkTableEntry mainTz = mainTable.getEntry("tz");
    NetworkTableEntry mainTa = mainTable.getEntry("ta");
    
    // April Limelight
    NetworkTable aprilTable = NetworkTableInstance.getDefault().getTable("limelight-april");
    NetworkTableEntry aprilTx = aprilTable.getEntry("tx");
    NetworkTableEntry aprilTy = aprilTable.getEntry("ty");
    NetworkTableEntry aprilTz = aprilTable.getEntry("tz");
    NetworkTableEntry aprilTa = aprilTable.getEntry("ta");

    // Limelight 4
    NetworkTable aprilTable4 = NetworkTableInstance.getDefault().getTable("limelight-april4");
    NetworkTableEntry april4Tx = aprilTable4.getEntry("tx");
    NetworkTableEntry april4Ty = aprilTable4.getEntry("ty");
    NetworkTableEntry april4Tz = aprilTable4.getEntry("tz");
    NetworkTableEntry april4Ta = aprilTable4.getEntry("ta");

    public AprilTagEstimator(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    
    @Override
    public void periodic() {
        processLimelight("limelight", mainTa);
        
        processLimelight("limelight-april", aprilTa);

        double mainArea = mainTa.getDouble(0.0);
        double aprilArea = aprilTa.getDouble(0.0);
        
        if (mainArea > 0 || aprilArea > 0) {
            bestLimelight = (mainArea > aprilArea) ? "limelight" : "limelight-april";
        } else {
            bestLimelight = "none";
        }
        
        SmartDashboard.putString("Best Limelight", bestLimelight);

    }

    
    
    private void processLimelight(String name, NetworkTableEntry ta) {
        SmartDashboard.putNumber(name + "/FiducialID", LimelightHelpers.getFiducialID(name));
        SmartDashboard.putBoolean(name + "/Target", LimelightHelpers.getTV(name));
        
        limeLightPose = LimelightHelpers.getBotPose3d_wpiBlue(name);
        limeLightPose2D = limeLightPose.toPose2d();
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        
        SmartDashboard.putNumber(name + "Area", ta.getDouble(0));

        if(limeLightPose != null){
            SmartDashboard.putNumber(name + "LimelightPoseX", limeLightPose.getX());
            SmartDashboard.putNumber(name + "LimelightPoseY", limeLightPose.getY());
            SmartDashboard.putNumber(name + "LimelightPoseZ", limeLightPose.getZ());

            SmartDashboard.putNumber(name + "LimelightPoseRotation", limeLightPose.getRotation().getAngle());
        }

        if(ta.getDouble(0) > GlobalVariables.limelightTolerance){
            updatePoseEstimatorWithVisionBotPose(limeLightPose2D, measurement, name);
        }
        
        
    }
    
    private void updatePoseEstimatorWithVisionBotPose(Pose2d limelightPose, 
                                                     LimelightHelpers.PoseEstimate limelightMeasurement,
                                                     String limelightName) {
        if(limelightPose.getX() == 0.0) return;
        
        double poseDifference = drivetrain.getPose().getTranslation()
            .getDistance(limelightPose.getTranslation());
            
        if (null != limelightPose && LimelightHelpers.getTV(limelightName)) {
            double xyStds;
            double degStds;
            
            if (limelightMeasurement.tagCount >= 2) {
                xyStds = 0.5;
                degStds = 6;
            } else if (limelightMeasurement.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            } else if (limelightMeasurement.avgTagArea > 0.3 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            } else {
                return;
            }
            
            drivetrain.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds)));
            
            double timestamp = Timer.getFPGATimestamp() - 
                (LimelightHelpers.getLatency_Pipeline(limelightName)/1000.0) - 
                (LimelightHelpers.getLatency_Capture(limelightName)/1000.0);
                
            drivetrain.addVisionMeasurement(limelightPose, timestamp);
        }
    }
    
}