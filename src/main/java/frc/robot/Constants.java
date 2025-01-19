package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public final class IDs {
        public static final int ELEVATOR_MOTOR = -1; // TODO: Determine Elevator Motor ID
    }

    public final class Elevator {
        public final class Gains {
            // From SysID routine
            public static final double kS = 0.0; // voltage to overcome static friction
            public static final double kG = 0.0; // voltage to overcome gravity
            public static final double kV = 0.0; // volts per 1 rps
            public static final double kA = 0.0; // volts per 1 rps/s

            // PID for correcting errors
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public final class ElevatorHeights {
            public static final double ELEVATOR_GEAR_RATIO = 0.0;

            // Min and Max Height for the Elevator
            public static final double ELEVATOR_MIN_HEIGHT = 0.0;
            public static final double ELEVATOR_MAX_HEIGHT = 0.0;
            public static final double TEST_HEIGHT = 0.0; // Should be removed later. Here for example purposes
            
            //TODO: find heights
            public static final double L1 = 1.0;
            public static final double L2 = 2.0;
            public static final double L3 = 3.0;
            public static final double L4 = 4.0;

            // TODO
            public static final double REACH_STATE_THRES = 0.1;
        }

        // Motion Profile
        public static final TrapezoidProfile ELEVATOR_MOTION_PROFILE = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                0.0, // max velocity in rps
                0.0 // max acceleration in rps/s
            )
        );

        // Converting from height to rotations

    }
}
