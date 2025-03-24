package us.kilroyrobotics;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    /**
     * Constants used for interfacing with drive subsystems and mechanisms that aren't autogenerated
     * from {@link us.kilroyrobotics.generated.TunerConstants TunerConstants}
     */
    public static final class DriveConstants {
        public static final LinearVelocity kLowDriveSpeed = MetersPerSecond.of(0.5);
        public static final LinearVelocity kMediumDriveSpeed = MetersPerSecond.of(2.0);
        public static final LinearVelocity kHighDriveSpeed = MetersPerSecond.of(3.5);
    }

    /** Constants used for interfacing with limelight vision */
    public static final class VisionConstants {
        public static final boolean kUseLimelight = true;

        public static final PIDController xPID = new PIDController(2.0, 5.0, 0.02);
        public static final PIDController yPID = new PIDController(2.0, 5.0, 0.02);
        public static final PIDController rotationalPID = new PIDController(2.0, 0.5, 0.02);

        public static final Pose2d kReefAPose =
                new Pose2d(Meters.of(2.99), Meters.of(4.19), new Rotation2d(Degrees.of(0)));
        public static final Pose2d kReefBPose =
                new Pose2d(Meters.of(2.99), Meters.of(3.87), new Rotation2d(Degrees.of(0)));
        public static final Pose2d kReefCPose =
                new Pose2d(Meters.of(3.58), Meters.of(2.82), new Rotation2d(Degrees.of(60)));
        public static final Pose2d kReefDPose =
                new Pose2d(Meters.of(3.84), Meters.of(2.63), new Rotation2d(Degrees.of(60)));
        public static final Pose2d kReefEPose =
                new Pose2d(Meters.of(5.08), Meters.of(2.65), new Rotation2d(Degrees.of(120)));
        public static final Pose2d kReefFPose =
                new Pose2d(Meters.of(5.34), Meters.of(2.78), new Rotation2d(Degrees.of(120)));
        public static final Pose2d kReefGPose =
                new Pose2d(Meters.of(5.96), Meters.of(3.85), new Rotation2d(Degrees.of(180)));
        public static final Pose2d kReefHPose =
                new Pose2d(Meters.of(5.98), Meters.of(4.17), new Rotation2d(Degrees.of(180)));
        public static final Pose2d kReefIPose =
                new Pose2d(Meters.of(5.37), Meters.of(5.22), new Rotation2d(Degrees.of(240)));
        public static final Pose2d kReefJPose =
                new Pose2d(Meters.of(5.14), Meters.of(5.39), new Rotation2d(Degrees.of(240)));
        public static final Pose2d kReefKPose =
                new Pose2d(Meters.of(3.92), Meters.of(5.40), new Rotation2d(Degrees.of(300)));
        public static final Pose2d kReefLPose =
                new Pose2d(Meters.of(3.61), Meters.of(5.29), new Rotation2d(Degrees.of(300)));

        /**
         * Get the alignment pose for the given april tag and left/right side of said april tag
         *
         * <p>The poses are flipped if the robot is on the red alliance, and the reef section names
         * will not match (e.g. Reef A -> Reef B, Reef C -> Reef L, etc...)
         *
         * <p>Additionally, it will make sure the april tag is for the correct alliance to prevent
         * any errors
         *
         * @param aprilTag the april tag of the target you want to align to
         * @param leftSide true if you want to get the left alignment pose for the given april tag
         * @param alliance the current alliance of the robot
         * @return the alignment pose for the given april tag (possibly null if no april tag or
         *     invalid april tag detected)
         */
        public static Pose2d getAlignmentPose(int aprilTag, boolean leftSide, Alliance alliance) {
            Pose2d pose = null;

            switch (aprilTag) {
                // Red Side
                case 7:
                    if (alliance == Alliance.Red)
                        pose =
                                leftSide
                                        ? FlippingUtil.flipFieldPose(kReefAPose)
                                        : FlippingUtil.flipFieldPose(kReefBPose);
                    break;
                case 8:
                    if (alliance == Alliance.Red)
                        pose =
                                leftSide
                                        ? FlippingUtil.flipFieldPose(kReefCPose)
                                        : FlippingUtil.flipFieldPose(kReefDPose);
                    break;
                case 9:
                    if (alliance == Alliance.Red)
                        pose =
                                leftSide
                                        ? FlippingUtil.flipFieldPose(kReefEPose)
                                        : FlippingUtil.flipFieldPose(kReefFPose);
                    break;
                case 10:
                    if (alliance == Alliance.Red)
                        pose =
                                leftSide
                                        ? FlippingUtil.flipFieldPose(kReefGPose)
                                        : FlippingUtil.flipFieldPose(kReefHPose);
                    break;
                case 11:
                    if (alliance == Alliance.Red)
                        pose =
                                leftSide
                                        ? FlippingUtil.flipFieldPose(kReefIPose)
                                        : FlippingUtil.flipFieldPose(kReefJPose);
                    break;
                case 6:
                    if (alliance == Alliance.Red)
                        pose =
                                leftSide
                                        ? FlippingUtil.flipFieldPose(kReefKPose)
                                        : FlippingUtil.flipFieldPose(kReefLPose);
                    break;

                // Blue Side
                case 18:
                    if (alliance == Alliance.Blue) pose = leftSide ? kReefAPose : kReefBPose;
                    break;
                case 17:
                    if (alliance == Alliance.Blue) pose = leftSide ? kReefCPose : kReefDPose;
                    break;
                case 22:
                    if (alliance == Alliance.Blue) pose = leftSide ? kReefEPose : kReefFPose;
                    break;
                case 21:
                    if (alliance == Alliance.Blue) pose = leftSide ? kReefGPose : kReefHPose;
                    break;
                case 20:
                    if (alliance == Alliance.Blue) pose = leftSide ? kReefIPose : kReefJPose;
                    break;
                case 19:
                    if (alliance == Alliance.Blue) pose = leftSide ? kReefKPose : kReefLPose;
                    break;
                default:
                    pose = null;
                    break;
            }

            return pose;
        }
    }

    /** Constants used for interfacing with the elevator subsystem */
    public static final class ElevatorConstants {
        /** The motor ID of the Spark Max associated with the left motor for the elevator */
        public static final int kLeftMotorId = 41;

        /** The motor ID of the Spark Max associated with the right motor for the elevator */
        public static final int kRightMotorId = 42;

        /** The % tolerance for determining when the elevator is in position */
        public static final double kPositionTolerance = 0.05;

        /** The zeroed motor encoder position in inches */
        public static final Distance kZeroed = Meters.of(Inches.of(13.1875).in(Meters));

        /** Height necessary for the coral intake to reach Level 1 of Reef in Meters */
        // 36.5
        public static final Distance kL1Height = Meters.of(Inches.of(25.6875).in(Meters));

        /** Height necessary for the coral intake to reach Level 2 of Reef in Meters */
        // 47.5
        public static final Distance kL2Height = Meters.of(Inches.of(39.5).in(Meters));

        /** Height necessary for the coral intake to reach Level 3 of Reef in Meters */
        public static final Distance kL3Height = Meters.of(Inches.of(55).in(Meters));

        /** Height necessary for the coral intake to reach Level 4 of Reef in Meters */
        public static final Distance kL4Height = Meters.of(Inches.of(68).in(Meters));

        /** Height necessary for the coral intake to reach the Coral Station */
        public static final Distance kCoralStationHeight = Meters.of(Inches.of(32.25).in(Meters));

        /* PIDF constants */
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.2;
        public static final double kF = 0.0;

        /**
         * Conversion factor which when multiplied by the raw encoder output results in the current
         * carriage height in meters
         */
        public static final double kEncoderPositionConversionFactor =
                Inches.of(11).in(Meters) / 9.0;

        // 1.0;

        /**
         * Conversion factor which when multiplied by the raw encoder output velocity results in the
         * current carriage velocity in meters/second
         */
        public static final double kEncoderVelocityConversionFactor = 1.0;

        /**
         * Conversion factor which takes the current encoder position to derive the height of the
         * second stage in meters.
         *
         * <p>1.0 + (inches carriage can move inside of 2nd stage / inches second stage can move
         * inside of 1st stage)
         */
        public static final double kSecondStagePositionConversionFactor =
                (1.0 + (Inches.of(26.0).in(Meters) / Inches.of(34.5).in(Meters)));

        public static final double kOverrideSpeedMultiplier = 0.25;

        public static final Distance kHeightLimit = Inches.of(50);
    }

    /** Constants used during Simulation */
    public static final class SimulationConstants {
        /** The gearing of the elevator gearbox */
        public static final double kElevatorGearing = 9.0;

        /** The weight in Kg of the carriage in Kilograms */
        public static final Mass kElevatorCarriageMass = Kilograms.of(1.0);

        /** The radius of the elevator rope drum in Meters */
        public static final Distance kElevatorDrumRadius = Meters.of(Inches.of(2.0).in(Meters));

        /** The minimum height of the elevator in Meters */
        public static final Distance kElevatorMinHeight = Meters.of(Inches.of(0.0).in(Meters));

        /** The maximum height of the elevator in Meters */
        public static final Distance kElevatorMaxHeight = Meters.of(Inches.of(74.5).in(Meters));

        /** The height of the elevator in Meters when the robot is in its starting configuration */
        public static final Distance kElevatorStartingHeight = Meters.of(Inches.of(0.0).in(Meters));

        /** The gearing of the wrist gearbox */
        public static final double kWristGearing = 64.0;

        /** The weight in Kg of the wrist in Kilograms */
        public static final Mass kWristMass = Kilograms.of(0.5);

        /** Arm length in meters */
        public static final Distance kArmLength = Meters.of(Inches.of(12.0).in(Meters));

        /** Min angle in Degrees */
        public static final Angle kMinAngle = Degrees.of(0);

        /** Max angle in Degrees */
        public static final Angle kMaxAngle = Degrees.of(180);
    }

    /** Constants used for interfacing with the Coral Intake and Wrist subsystems */
    public static final class CoralMechanismConstants {
        /** The motor ID of the Spark Max associated with the wrist motor for the coral mechanism */
        public static final int kWristMotorId = 43;

        /** The motor ID of the Spark Max associated with the motor for the coral intake */
        public static final int kWheelMotorId = 44;

        /** The % tolerance for determining when the wrist is in position */
        public static final double kAngleTolerance = 0.05;

        /* Wrist Configurations */
        /** The angle that the wrist will be at the start of the match */
        public static final Angle kStartingAngle = Degrees.of(0);

        /** The angle that the wrist will be when intaking a coral */
        public static final Angle kIntakingAngle = Degrees.of(37.5);

        /** The angle that the wrist will be when scoring at L4 */
        public static final Angle kScoringL4 = Degrees.of(120);

        /** The angle that the wrist will be when scoring at L3 */
        public static final Angle kScoringL3 = Degrees.of(110);

        /** The angle that the wrist will be when scoring at L2 */
        public static final Angle kScoringL2 = Degrees.of(110);

        /** The angle that the wrist will be when scoring at L1 */
        public static final Angle kScoringL1 = Degrees.of(120);

        /* PIDF constants */
        public static final double kP = 1.5;
        public static final double kI = 0.0;
        public static final double kD = 0.05;
        public static final double kF = 0.0;

        /* Wheel Speeds */
        /** Speed of the motor when intaking a coral piece */
        public static final double kWheelSpeedIntaking = -0.45;

        /** Speed of the motor when scoring a coral piece */
        public static final double kWheelSpeedOuttaking = 0.2;

        /** Speed of the motor while the robot is in motion to keep the coral piece in place */
        public static final double kWheelSpeedHolding = -0.2;

        public static final double kOverrideSpeedMultiplier = 0.25;
    }

    public static final class AlgaeConstants {

        /** Algae motor IDs */
        public static final int kAlgaeMotorLeaderId = 45;

        public static final int kAlgaeMotorFollowerId = 46;

        public static final double kAlgaeSpeedIntaking = 0.75;
        public static final double kAlgaeSpeedOuttaking = -0.75;
    }

    public static final class CameraConstants {
        /* SOFTWARE PROPERTIES */
        public static final boolean kCameraEnabled = false;

        /* CAMERA PROPERTIES */
        public static final int[] kResolution = {640, 480};
        public static final int kFPS = 10;
        public static final int kCompression = 60;
        public static final int kBrightness = 35;
    }
}
