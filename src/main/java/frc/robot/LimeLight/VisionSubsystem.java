package frc.robot.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.HashMap;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{

    public static final String LimeLight1 = "limelight"; //CAMBIAR NOMBRE

    // Adjustable transform for the Limelight pose per-alliance
    @SuppressWarnings("unused")
    private static final Transform2d LL_BLUE_TRANSFORM = new Transform2d(0, 0, new Rotation2d());
    @SuppressWarnings("unused")
    private static final Transform2d LL_RED_TRANSFORM = new Transform2d(0, 0, new Rotation2d());

    public AprilTagFieldLayout aprilTagFieldLayout;
    private boolean odometryEnabled = true;
    private double lastOdometryTime = 0;

    private double limelightHeartbeat = 0;
    @SuppressWarnings("unused")
    private double frontLimelightHeartbeat = 0;
    private double lastHeartbeatTime = 0;
    @SuppressWarnings("unused")
    private double frontLastHeartbeatTime = 0;
    private boolean limelightConnected = false;
    @SuppressWarnings("unused")
    private boolean frontLimelightConnected = false;

    public Double distanceOverride = null;
    @SuppressWarnings("unused")
    private Double tagHeightOverride = null;
    @SuppressWarnings("unused")
    private PoseEstimate lastPoseEstimate = null;
    private Double stageAngle = null;
    private Pose2d currentTrapPose = null;
    public Double stageTX = null;
    public Double stageTY = null;
    private boolean megatag2Enabled = false;
    public HashMap<Integer, Pose2d> trapPoses = new HashMap<>();

    private Pose2d speakerPoseBlue = null;
    private Pose2d speakerPoseRed = null;


    private final RawFiducial[] emptyFiducials = new RawFiducial[0];
    public RawFiducial[] rawFiducials = emptyFiducials;

    private boolean firstPeriodic = true;
    private final Transform2d trapPoseTransform = new Transform2d(1.132, 0.32, Rotation2d.fromRadians(-0.005));
    private final Pose2d nilPose = new Pose2d(-1, -1, new Rotation2d());

    final int[] autoTagFilter = new int[] { 3, 4, 7, 8 };
    final int[] teleopTagFilter = new int[] { 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16 };
    public boolean isBlue(){
        return DriverStation.getAlliance().get()==Alliance.Blue;
        }public boolean isRed(){
            return DriverStation.getAlliance().get()==Alliance.Red;
            }
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        SmartDashboard.putNumber("BLUE Distance Offset", 0);
        SmartDashboard.putNumber("RED Distance Offset", 0);

        SmartDashboard.putNumber("RED Pixel Offset", 0);
        SmartDashboard.putNumber("BLUE Pixel Offset", 0);

        for (int i = 11; i <= 16; i++) {
            trapPoses.put(i, calculateTrapPose(i));
        }
        currentTrapPose = trapPoses.get(11);

        
    }

    @Override
    public void periodic() {
        if (firstPeriodic) {
            for (int i = 11; i <= 16; i++) {
                //log("Trap Poses/" + i, trapPoses.get(i));
            }
            RobotContainer.drivetrain.addVisionMeasurement(new Pose2d(), Timer.getFPGATimestamp());
            firstPeriodic = false;
        }

        SmartDashboard.putBoolean("Odometry Enabled", odometryEnabled);     
        
        double newHeartbeat = LimelightHelpers.getLimelightNTDouble("limelight", "hb");
        if (newHeartbeat > limelightHeartbeat) {
            limelightConnected = true;
            limelightHeartbeat = newHeartbeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - lastHeartbeatTime >= 1) {
            limelightConnected = false;
        }
        if (Robot.isSimulation()) limelightConnected = true;
        
        SmartDashboard.putBoolean("LimeLight Is Connected", limelightConnected);

        if (!limelightConnected) {
            rawFiducials = emptyFiducials;
            
            return;
        }


        PoseEstimate bestPose;
        PoseEstimate frontPose = validatePoseEstimate(null, 0); // Not using front limelight for odometry yet
        PoseEstimate backPose;
        PoseEstimate megaTag2Pose = null;

        
        if (megatag2Enabled) {
            double megatagDegrees = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
            if (DriverStation.getAlliance().get()==Alliance.Red) megatagDegrees = MathUtil.inputModulus(megatagDegrees + 180, -180, 180);
            LimelightHelpers.SetRobotOrientation(LimeLight1, megatagDegrees, 0, 0, 0, 0, 0);
            backPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimeLight1);
        } else {
            backPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLight1);
        }

        if (backPose != null)  {
            rawFiducials = backPose.rawFiducials;
        } else {
            rawFiducials = emptyFiducials;
        }

        double deltaSeconds = Timer.getFPGATimestamp() - lastOdometryTime;
        
        SmartDashboard.putNumber("LL Pose Pre-Validation : X Axis ", backPose == null ? nilPose.getX() : backPose.pose.getX());       
        SmartDashboard.putNumber("LL Pose Pre-Validation : Y Axis ", backPose == null ? nilPose.getY() : backPose.pose.getY());       
        SmartDashboard.putNumber("LL Pose Pre-Validation : Rotation Degrees ", backPose == null ? nilPose.getRotation().getDegrees() : backPose.pose.getRotation().getDegrees());       

        SmartDashboard.putNumber(" LL MegaTag2 : X Axis ", megaTag2Pose == null ? nilPose.getX() : megaTag2Pose.pose.getX() );
        SmartDashboard.putNumber(" LL MegaTag2 : Y Axis ", megaTag2Pose == null ? nilPose.getY() : megaTag2Pose.pose.getY() );
        SmartDashboard.putNumber(" LL MegaTag2 : Rotation Degrees ", megaTag2Pose == null ? nilPose.getRotation().getDegrees() : megaTag2Pose.pose.getRotation().getDegrees() );
        
        backPose = validatePoseEstimate(backPose, deltaSeconds);
        
        if (frontPose != null && backPose != null) {
            bestPose = (frontPose.avgTagArea >= backPose.avgTagArea) ? frontPose : backPose;
        } else if (frontPose != null) {
            bestPose = frontPose;
        } else {
            bestPose = backPose;
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
        LimelightHelpers.SetFiducialIDFiltersOverride(LimeLight1, autoTagFilter);
    }

    public void setTeleopTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(LimeLight1, teleopTagFilter);
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

    public Pose2d getSpeakerPose() {
        return isBlue() ? speakerPoseBlue : speakerPoseRed;
    }


    public boolean isLimelightConnected() {
        return limelightConnected;
    }

    public Pose2d calculateTrapPose(int tagID) {
        Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID);
        if (pose3d.isEmpty()) return null;

        return pose3d.get().toPose2d().transformBy(trapPoseTransform);
    }

    public Pose2d getCurrentTrapPose() {
        return currentTrapPose;
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

    public Double getStageAngle() {
        return stageAngle;
    }

    public boolean hasStageAngle() {
        return stageAngle != null;
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
