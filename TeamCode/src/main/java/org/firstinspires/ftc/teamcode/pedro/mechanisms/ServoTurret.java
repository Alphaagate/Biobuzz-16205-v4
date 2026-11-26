
package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoTurret {

    private DcMotorEx turretMotor;

    // =========================================
    // FIELD TARGET: HIVE
    // =========================================

    public static double GOAL_X = 60.0;
    public static double GOAL_Y = 60.0;


    // =========================================
    // PHYSICAL CONFIGURATION
    // =========================================

    public static double TURRET_OFFSET_FROM_ROBOT_CENTER = 1.0;

    // =========================================
    // MOTOR AND GEARING
    // =========================================

    // goBILDA 435 RPM Yellow Jacket
    // Encoder resolution at motor gearbox output.

    public static double MOTOR_TICKS_PER_REV = 384.5;

    // External gear teeth
    public static final double TURRET_GEAR_TEETH = 80.0;
    public static final double MOTOR_GEAR_TEETH = 27.0;

    // Motor rotations per turret rotation
    public static final double GEAR_RATIO =
            TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH;


    // =========================================
    // SOFT LIMITS
    // =========================================

    // 0 degrees = forward
    // Positive = counterclockwise

    public static double MIN_ANGLE_DEG = -180.0;
    public static double MAX_ANGLE_DEG = 180.0;


    // =========================================
    // MOTOR TUNING
    // =========================================

    public static double MAX_POWER = 0.6;

    public static double POSITION_P = 10.0;

    public static int TOLERANCE_TICKS = 8;

    public static double TRIM_DEG = 0.0;

    public static double HEADING_OFFSET = 0.0;

    public static double HEADING_OFFSET_STEP =
            Math.toRadians(2.0);


    // =========================================
    // STATE
    // =========================================

    private double distanceToGoal = 0.0;

    private int targetTicks = 0;

    private double targetAngleDeg = 0.0;

    private Pose currentRobotPose = null;


    // =========================================
    // INITIALIZATION
    // =========================================

    public void init(HardwareMap hwMap) {

        turretMotor = hwMap.get(
                DcMotorEx.class,
                "turret"
        );

        // Change to REVERSE if positive encoder
        // movement does not correspond to CCW.

        turretMotor.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // IMPORTANT:
        // Physically center the turret before initializing.
        // Its forward-facing position becomes encoder zero.

        turretMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        turretMotor.setTargetPosition(0);

        turretMotor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        turretMotor.setPositionPIDFCoefficients(
                POSITION_P
        );

        turretMotor.setPower(MAX_POWER);
    }


    // =========================================
    // AUTOMATIC FIELD TARGETING
    // =========================================

    public void update(Follower follower) {

        aimAtFieldPoint(
                follower.pose(),
                GOAL_X,
                GOAL_Y
        );
    }


    public void aimAtFieldPoint(
            Pose robotPose,
            double goalX,
            double goalY
    ) {

        double dx = goalX - getTurretX(robotPose);

        double dy = goalY - getTurretY(robotPose);

        double distance = Math.hypot(dx, dy);

        double fieldAngle = Math.atan2(dy, dx);

        aimAtFieldAngle(
                robotPose,
                fieldAngle,
                distance
        );
    }


    // =========================================
    // TURRET POSITION CONTROL
    // =========================================


    public void aimAtFieldAngle(
            Pose robotPose,
            double fieldAngle,
            double distance
    ) {

        currentRobotPose = robotPose;
        distanceToGoal = distance;

        // Calculate the desired angle relative to the robot
        double relative = normalizeAngle(
                fieldAngle - robotPose.heading()
        );

        // Apply calibration offsets
        relative = normalizeAngle(
                relative + HEADING_OFFSET
        );

        double desiredDeg =
                Math.toDegrees(relative) + TRIM_DEG;

        // Read the actual turret encoder angle.
        // Do NOT normalize this angle.
        // It can exceed 360 degrees on a continuous turret.
        double currentDeg = getTurretAngleDegrees();

        // Find the equivalent target requiring the shortest rotation.
    // Calculate shortest rotation.
    // Range: -180 degrees to +180 degrees.
    // Exactly 180 degrees always chooses the negative direction.
        double deltaDeg = desiredDeg - currentDeg;

        while (deltaDeg >= 180.0) {
            deltaDeg -= 360.0;
        }

        while (deltaDeg < -180.0) {
            deltaDeg += 360.0;
        }

        double bestTarget = currentDeg + deltaDeg;

        targetAngleDeg = bestTarget;

        // Convert turret degrees to motor encoder ticks
        targetTicks = (int) Math.round(
                bestTarget * ticksPerDegree()
        );

        // Command the turret
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(MAX_POWER);
    }

    public int getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    // =========================================
    // TURRET POSITION
    // =========================================

    public double getTurretX(Pose p) {

        return p.x()
                - TURRET_OFFSET_FROM_ROBOT_CENTER
                * Math.cos(p.heading());
    }


    public double getTurretY(Pose p) {

        return p.y()
                - TURRET_OFFSET_FROM_ROBOT_CENTER
                * Math.sin(p.heading());
    }


    // =========================================
    // ENCODER CONVERSION
    // =========================================

    private double ticksPerDegree() {

        return (
                MOTOR_TICKS_PER_REV * GEAR_RATIO
        ) / 360.0;
    }


    // =========================================
    // MOTOR CONTROL
    // =========================================

    public void stop() {

        turretMotor.setPower(0);
    }


    public boolean atTarget() {

        return Math.abs(
                turretMotor.getCurrentPosition()
                        - targetTicks
        ) <= TOLERANCE_TICKS;
    }


    // =========================================
    // MANUAL CALIBRATION
    // =========================================

    public void adjustTrim(double deltaDeg) {

        TRIM_DEG += deltaDeg;
    }


    public void adjustHeadingOffset(double d) {

        HEADING_OFFSET += d;
    }


    public void resetHeadingOffset() {

        HEADING_OFFSET = 0.0;
    }


    // =========================================
    // ANGLE NORMALIZATION
    // =========================================

    private double normalizeAngle(double a) {

        while (a > Math.PI) {
            a -= 2.0 * Math.PI;
        }

        while (a < -Math.PI) {
            a += 2.0 * Math.PI;
        }

        return a;
    }


    // =========================================
    // TELEMETRY
    // =========================================

    public double getDistanceToGoal() {

        return distanceToGoal;
    }


    public int getTargetTicks() {

        return targetTicks;
    }


    public double getTargetAngleDegrees() {

        return targetAngleDeg;
    }


    public double getTurretAngleDegrees() {

        return turretMotor.getCurrentPosition()
                / ticksPerDegree();
    }


    public Pose getCurrentRobotPose() {

        return currentRobotPose;
    }
}
