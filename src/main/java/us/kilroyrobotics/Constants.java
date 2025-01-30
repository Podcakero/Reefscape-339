package us.kilroyrobotics;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public final class Constants {
    /**
     * Constants used for interfacing with drive subsystems and mechanisms that aren't autogenerated
     * from {@link us.kilroyrobotics.generated.TunerConstants TunerConstants}
     */
    public static final class DriveConstants {
        public static final LinearVelocity kLowDriveSpeed = MetersPerSecond.of(0.5);
    }

    /** Constants used for interfacing with limelight vision */
    public static final class VisionConstants {
        public static final boolean kUseLimelight = false;
    }

    /** Constants used for interfacing with the elevator subsystem */
    public static final class ElevatorConstants {
        /** The motor ID of the Spark Max associated with the left motor for the elevator */
        public static final int kLeftMotorId = 41;

        /** The motor ID of the Spark Max associated with the right motor for the elevator */
        public static final int kRightMotorId = 42;

        /** Height necessary for the coral intake to reach Level 1 of Reef in Meters */
        public static final Distance kL1Height = Meters.of(Inches.of(11.5).in(Meters));

        /** Height necessary for the coral intake to reach Level 2 of Reef in Meters */
        public static final Distance kL2Height = Meters.of(Inches.of(17.5).in(Meters));

        /** Height necessary for the coral intake to reach Level 3 of Reef in Meters */
        public static final Distance kL3Height = Meters.of(Inches.of(29.5).in(Meters));

        /** Height necessary for the coral intake to reach Level 4 of Reef in Meters */
        public static final Distance kL4Height = Meters.of(Inches.of(60.5).in(Meters));

        /** Height necessary for the coral intake to reach the Coral Station */
        public static final Distance kCoralStationHeight = Meters.of(Inches.of(19.5).in(Meters));

        /* PIDF constants */
        public static final double kP = 5.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        /**
         * Conversion factor which when multiplied by the raw encoder output results in the current
         * carriage height in meters
         *
         * <p>gear ratio * sprocket OD * PI = Meters per rotation
         */
        public static final double kEncoderPositionConversionFactor =
                Inches.of(1.0 / 9.0 * 15.0 / 8.0 * Math.PI).in(Meters);

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
    }

    /** Constants used for interfacing with the Coral Intake and Wrist subsystems */
    public static final class CoralMechanismConstants {
        /** The motor ID of the Spark Max associated with the wrist motor for the coral mechanism */
        public static final int kWristMotorId = 43;

        /** The motor ID of the Spark Max associated with the motor for the coral intake */
        public static final int kWheelMotorId = 44;

        /* Wrist Configurations */
        /** The angle that the wrist will be at the start of the match */
        public static final Angle kStartingAngle = Degrees.of(0);

        /** The angle that the wrist will be when intaking a coral */
        public static final Angle kIntakingAngle = Degrees.of(45);

        /** The angle that the wrist will be when scoring at L4 */
        public static final Angle kScoringHighLevel = Degrees.of(180);

        /** The angle that the wrist will be when scoring at L2-L3 */
        public static final Angle kScoringMidLevel = Degrees.of(110);

        /** The angle that the wrist will be when scoring at L1 */
        public static final Angle kScoringLowLevel = Degrees.of(160);

        /* Wheel Speeds */
        /** Speed of the motor when intaking a coral piece */
        public static final double kWheelSpeedIntaking = 0.75;

        /** Speed of the motor when scoring a coral piece */
        public static final double kWheelSpeedOuttaking = -0.75;

        /** Speed of the motor while the robot is in motion to keep the coral piece in place */
        public static final double kWheelSpeedHolding = 0.15;
    }
}
