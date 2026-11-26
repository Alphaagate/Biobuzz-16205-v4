
package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    private final Servo hood;

    // =========================================
    // BALLISTIC CONFIGURATION
    // =========================================

    // Replace these with your measured dimensions (inches).
    public static double TARGET_HEIGHT_IN = 0.0;
    public static double SHOOTER_EXIT_HEIGHT_IN = 0.0;

    // Desired angle at which the ball enters the target.
    public static double TARGET_ENTRY_ANGLE_DEGREES = 0.0;

    // =========================================
    // HOOD MECHANICAL LIMITS
    // =========================================

    // Example values. Calibrate these for your mechanism.
    public static double MIN_LAUNCH_ANGLE_DEG = 20.0;
    public static double MAX_LAUNCH_ANGLE_DEG = 70.0;

    // Servo positions corresponding to the two launch-angle limits.
    // These are calibration values, NOT universal servo positions.
    public static double SERVO_AT_MIN_ANGLE = 0.30;
    public static double SERVO_AT_MAX_ANGLE = 0.70;

    private double commandedAngle = 0;
    private double servoPosition = 0.5;

    public Hood(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");
    }

    // =========================================
    // BALLISTIC CALCULATION
    // =========================================
    public static final double HOLD = 0.5;



    public void hold() {
        hood.setPosition(HOLD);
    }



    public CommandBuilder holdhood() { return Commands.instant(this::hold); }

//    public double calculateLaunchAngle(double distance) {
//
//        if (!Double.isFinite(distance) || distance <= 0) {
//            throw new IllegalArgumentException(
//                    "Distance must be greater than zero."
//            );
//        }
//
//        double height =
//                TARGET_HEIGHT_IN - SHOOTER_EXIT_HEIGHT_IN;
//
//        double entryAngle =
//                Math.toRadians(TARGET_ENTRY_ANGLE_DEGREES);
//
//        double launchAngle = Math.atan(
//                2.0 * height / distance - Math.tan(entryAngle)
//        );
//
//        return Math.toDegrees(launchAngle);
//    }

    // =========================================
    // AUTOMATIC HOOD CONTROL
    // =========================================

//    public void aimForDistance(double distance) {
//
//        double angle = calculateLaunchAngle(distance);
//
//        setLaunchAngleDegrees(angle);
//    }

//    public void setLaunchAngleDegrees(double requestedAngle) {
//
//        // Keep the hood within its mechanical limits.
//        double angle = clamp(
//                requestedAngle,
//                MIN_LAUNCH_ANGLE_DEG,
//                MAX_LAUNCH_ANGLE_DEG
//        );
//
//        commandedAngle = angle;
//
//        // Convert launch angle into a normalized servo position.
//        double fraction =
//                (angle - MIN_LAUNCH_ANGLE_DEG) /
//                        (MAX_LAUNCH_ANGLE_DEG - MIN_LAUNCH_ANGLE_DEG);
//
//        double position =
//                SERVO_AT_MIN_ANGLE +
//                        fraction * (SERVO_AT_MAX_ANGLE - SERVO_AT_MIN_ANGLE);
//
//        // Stay inside the calibrated servo range.
//        double low = Math.min(
//                SERVO_AT_MIN_ANGLE,
//                SERVO_AT_MAX_ANGLE
//        );
//
//        double high = Math.max(
//                SERVO_AT_MIN_ANGLE,
//                SERVO_AT_MAX_ANGLE
//        );
//
//        servoPosition = clamp(position, low, high);
//
//        hood.setPosition(servoPosition);
//    }

    // =========================================
    // HELPERS
    // =========================================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getCommandedAngle() {
        return commandedAngle;
    }

    public double getServoPosition() {
        return servoPosition;
    }
}
