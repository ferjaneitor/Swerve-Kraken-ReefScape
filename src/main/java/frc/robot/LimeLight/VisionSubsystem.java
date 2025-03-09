package frc.robot.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{

    public static final String LimeLight_UP = "P_Sherman_D"; //CAMBIAR NOMBRE
    public static final String LimeLight_Down = "P_Sherman_U"; //CAMBIAR NOMBRE

    // Adjustable transform for the Limelight pose per-alliance
    @SuppressWarnings("unused")
    private static final Transform2d LL_BLUE_TRANSFORM = new Transform2d(0, 0, new Rotation2d());
    @SuppressWarnings("unused")
    private static final Transform2d LL_RED_TRANSFORM = new Transform2d(0, 0, new Rotation2d());

    public AprilTagFieldLayout aprilTagFieldLayout;
    private boolean odometryEnabled = true;
    private double lastOdometryTime = 0;

    private double LimeLightUPHeartbeat = 0;
    private double LimeLightDOWNHeartbeat = 0;
    @SuppressWarnings("unused")
    private double frontLimeLightUPHeartbeat = 0;
    @SuppressWarnings("unused")
    private double frontLimeLightDOWNHeartbeat = 0;
    private double lastHeartUPbeatTime = 0;
    private double lastHeartDOWNbeatTime = 0;
    @SuppressWarnings("unused")
    private double frontLastHeartUPbeatTime = 0;
    private boolean LimeLightUPConnected = false;
    private boolean LimeLightDOWNConnected = false;
    @SuppressWarnings("unused")
    private boolean frontLimeLightUPConnected = false;

    public Double distanceOverride = null;
    @SuppressWarnings("unused")
    private Double tagHeightOverride = null;
    @SuppressWarnings("unused")
    private PoseEstimate lastPoseEstimate = null;
    public Double stageTX = null;
    public Double stageTY = null;
    private boolean megatag2Enabled = false;

    private final RawFiducial[] emptyFiducials = new RawFiducial[0];
    public RawFiducial[] rawFiducials = emptyFiducials;

    private boolean firstPeriodic = true;
    private final Pose2d nilPose = new Pose2d(-1, -1, new Rotation2d());

    final int[] autoTagFilter = new int[] { 1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22 };
    final int[] teleopTagFilter = new int[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };
    public boolean isBlue(){
        return DriverStation.getAlliance().get()==Alliance.Blue;
        }public boolean isRed(){
            return DriverStation.getAlliance().get()==Alliance.Red;
            }
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);;
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        SmartDashboard.putNumber("BLUE Distance Offset", 0);
        SmartDashboard.putNumber("RED Distance Offset", 0);

        SmartDashboard.putNumber("RED Pixel Offset", 0);
        SmartDashboard.putNumber("BLUE Pixel Offset", 0);

    }

    @Override
    public void periodic() {

        if (firstPeriodic) {
            
            RobotContainer.drivetrain.addVisionMeasurement(new Pose2d(), Timer.getFPGATimestamp());
            firstPeriodic = false;

        }

        SmartDashboard.putBoolean("Odometry Enabled", odometryEnabled);     
        
        double newUPHeartbeat = LimelightHelpers.getLimelightNTDouble("limelight-up", "hb");
        
        double newDownHeartbeat = LimelightHelpers.getLimelightNTDouble("limelight-down", "hb");

        if (newUPHeartbeat > LimeLightUPHeartbeat) {

            LimeLightUPConnected = true;
            LimeLightUPHeartbeat = newUPHeartbeat;


            lastHeartUPbeatTime = Timer.getFPGATimestamp();

        } else if (Timer.getFPGATimestamp() - lastHeartUPbeatTime >= 1) {

            LimeLightUPConnected = false;
        }

        if (newDownHeartbeat > LimeLightDOWNHeartbeat) {

            LimeLightDOWNConnected = true;
            LimeLightDOWNHeartbeat = newDownHeartbeat;


            lastHeartDOWNbeatTime = Timer.getFPGATimestamp();

        } else if (Timer.getFPGATimestamp() - lastHeartDOWNbeatTime >= 1) {

            LimeLightDOWNConnected = false;
        }

        if (Robot.isSimulation()) {
            LimeLightUPConnected = true;
            LimeLightDOWNConnected = true;
        }
        
        SmartDashboard.putBoolean("LimeLight-UP Is Connected", LimeLightUPConnected);
        SmartDashboard.putBoolean("LimeLight-DOWN Is Connected", LimeLightDOWNConnected);

        if (!LimeLightUPConnected && !LimeLightDOWNConnected) {

            rawFiducials = emptyFiducials;
            
            return;

        }

        PoseEstimate bestPose;
        PoseEstimate frontPose = validatePoseEstimate(null, 0); // Not using front limelight for odometry yet
        PoseEstimate upPose;
        PoseEstimate downPose;
        PoseEstimate megaTag2Pose = null;

        
        if (megatag2Enabled) {
            
            double megatagDegrees = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
            if (DriverStation.getAlliance().get()==Alliance.Red) megatagDegrees = MathUtil.inputModulus(megatagDegrees + 180, -180, 180);
            LimelightHelpers.SetRobotOrientation(LimeLight_UP, megatagDegrees, 0, 0, 0, 0, 0);
            upPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimeLight_UP);
            
            LimelightHelpers.SetRobotOrientation(LimeLight_Down, megatagDegrees, 0, 0, 0, 0, 0);
            downPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimeLight_Down);
            
        } else {
            
            upPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLight_UP);
            
            downPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLight_Down);
            
        }

        double deltaSeconds = Timer.getFPGATimestamp() - lastOdometryTime;
        
        SmartDashboard.putNumber("LL UP Pose Pre-Validation : X Axis ", upPose == null ? nilPose.getX() : upPose.pose.getX());       
        SmartDashboard.putNumber("LL UP Pose Pre-Validation : Y Axis ", upPose == null ? nilPose.getY() : upPose.pose.getY());       
        SmartDashboard.putNumber("LL UP Pose Pre-Validation : Rotation Degrees ", upPose == null ? nilPose.getRotation().getDegrees() : upPose.pose.getRotation().getDegrees());       
        
        SmartDashboard.putNumber("LL DOWN Pose Pre-Validation : X Axis ", downPose == null ? nilPose.getX() : downPose.pose.getX());       
        SmartDashboard.putNumber("LL DOWN Pose Pre-Validation : Y Axis ", downPose == null ? nilPose.getY() : downPose.pose.getY());       
        SmartDashboard.putNumber("LL DOWN Pose Pre-Validation : Rotation Degrees ", downPose == null ? nilPose.getRotation().getDegrees() : downPose.pose.getRotation().getDegrees());       

        SmartDashboard.putNumber(" LL MegaTag2 : X Axis ", megaTag2Pose == null ? nilPose.getX() : megaTag2Pose.pose.getX() );
        SmartDashboard.putNumber(" LL MegaTag2 : Y Axis ", megaTag2Pose == null ? nilPose.getY() : megaTag2Pose.pose.getY() );
        SmartDashboard.putNumber(" LL MegaTag2 : Rotation Degrees ", megaTag2Pose == null ? nilPose.getRotation().getDegrees() : megaTag2Pose.pose.getRotation().getDegrees() );
        
        downPose = validatePoseEstimate(downPose, deltaSeconds);
        
        upPose = validatePoseEstimate(upPose, deltaSeconds);
        
        if (frontPose != null && upPose != null) {
            bestPose = (downPose.avgTagArea >= upPose.avgTagArea) ? downPose : upPose;
        } else if (downPose != null) {
            bestPose = downPose;
        } else {
            bestPose = upPose;
        }
        
        if (bestPose != null) {
            lastOdometryTime = Timer.getFPGATimestamp();
            lastPoseEstimate = bestPose;
            if (odometryEnabled) {
                RobotContainer.drivetrain.addVisionMeasurement(bestPose.pose, bestPose.timestampSeconds);
            }
        }

        SmartDashboard.putBoolean("LL BestPose Is Valid ?", bestPose != null);
  
        SmartDashboard.putNumber("LL Pose : X Axis", bestPose == null ? nilPose.getX() : bestPose.pose.getX());      
        SmartDashboard.putNumber("LL Pose : Y Axis", bestPose == null ? nilPose.getY() : bestPose.pose.getY());      
        SmartDashboard.putNumber("LL Pose : Rotation Degrees", bestPose == null ? nilPose.getRotation().getDegrees() : bestPose.pose.getRotation().getDegrees());      
  
        SmartDashboard.putNumber("LL Pose Avarage Target Distance", bestPose == null ? -1 : bestPose.avgTagDist);
        
        SmartDashboard.putNumber("LL POse Avarage Target Area", bestPose == null ? -1 : bestPose.avgTagDist);
  
    }

    public void setAutoTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(LimeLight_UP, autoTagFilter);
    }

    public void setTeleopTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(LimeLight_UP, teleopTagFilter);
    }

    public RawFiducial getFiducialRange(int low, int high) {
        for (RawFiducial tag : rawFiducials) {
            if (tag != null && low <= tag.id && tag.id <= high) {
                return tag;
            }
        }

        return null;
    }

    public RawFiducial getBiggestFiducialRange(int low, int high) {
        RawFiducial biggest = null;
        double area = -1;
        for (RawFiducial tag : rawFiducials) {
            if (tag != null && low <= tag.id && tag.id <= high) {
                if (biggest == null || tag.ta > area) {
                    biggest = tag;
                    area = tag.ta;
                }
            }
        }
        return biggest;
    }

    public boolean isLimeLightUPConnected() {
        return LimeLightUPConnected;
    }
    
    public boolean isLimeLightDOWNConnected() {
        return LimeLightDOWNConnected;
    }
    
    public void setAllowVisionOdometry(boolean odometryEnabled) {
        this.odometryEnabled = odometryEnabled;
    }
    
    public Command allowVisionOdometry(boolean odometryEnabled) {
        return Commands.runOnce(() -> setAllowVisionOdometry(odometryEnabled));
    }

    public Double getDistanceOverride() {
        return distanceOverride;
    }

    public void setDistanceOverride(Double distance) {
        distanceOverride = distance;
    }

    public Command clearDistanceOverride() {
        return distanceOverride(null);
    }

    public Command distanceOverride(Double distance) {
        return Commands.runOnce(() -> setDistanceOverride(distance));
    }

    public void setTagHeightOverride(Double height) {
        tagHeightOverride = height;
    }

    public Command clearTagHeightOverride() {
        return tagHeightOverride(null);
    }

    public Command tagHeightOverride(Double height) {
        return Commands.runOnce(() -> setTagHeightOverride(height));
    }

    public void setMegatag2Enabled(boolean enabled) {
        megatag2Enabled = enabled;
    }

    public Command megatag2Enabled(boolean enabled) {
        return Commands.runOnce(() -> setMegatag2Enabled(enabled));
    }

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate, double deltaSeconds) {
        if (poseEstimate == null) return null;

        if (megatag2Enabled) {
            if (poseEstimate.tagCount == 0) return null;
            if (Math.abs(RobotContainer.drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) return null;
        } else {
            double tagMin = 1;
            double tagMax = 2;
            double minArea = 0.08;
            if (poseEstimate.tagCount == 1) minArea = 0.18;
            if (poseEstimate.tagCount > tagMax || poseEstimate.tagCount < tagMin) return null;
            if (poseEstimate.avgTagArea < minArea) return null;
            if (poseEstimate.avgTagDist > 6) return null;

        }

        return poseEstimate;
    }

    public class LimelightPose {
        public final Pose2d pose;
        public final double timestamp;
        public final double targetArea;

        public LimelightPose(Pose2d pose, double timestamp, double targetArea) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.targetArea = targetArea;
        }
    }
}
