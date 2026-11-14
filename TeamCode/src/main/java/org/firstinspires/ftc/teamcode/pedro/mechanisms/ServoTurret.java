package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class ServoTurret {

    private Servo turretA, turretB;

    // ========================================================================
    // FIELD CONFIGURATION
    // ========================================================================

    public static double GOAL_X = 60.0;
    public static double GOAL_Y = 60;

    // ========================================================================
    // PHYSICAL CONFIGURATION
    // ========================================================================

    public static double TURRET_OFFSET_FROM_ROBOT_CENTER = 1.0;

    public static final double SERVO_DEGREES = 355.0;
    public static final double SERVO_MIN = 0.015;
    public static final double SERVO_MAX = 0.95;
    public static final double DEFAULT_POSITION = 0.5;

    // Fine-tune alignment without touching the math (configurable via dashboard)
    public static double SERVO_TRIM = 0.0;
    public static double MANUAL_TRIM_STEP = 0.005;

    // Angular offset applied before servo math — use buttons to nudge aim left/right
    public static double HEADING_OFFSET = 0.0; // radians
    public static double HEADING_OFFSET_STEP = Math.toRadians(2.0); // 2° per press

    // ========================================================================
    // STATE
    // ========================================================================

    private double distanceToGoal = 0;
    private double targetServoPosition = DEFAULT_POSITION;
    private Pose currentRobotPose = null;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    public void init(HardwareMap hwMap) {
        turretA = hwMap.get(Servo.class, "turretA");
        turretB = hwMap.get(Servo.class, "turretB");

        turretA.setDirection(Servo.Direction.FORWARD);
        turretB.setDirection(Servo.Direction.FORWARD);

        setExtendedPwmRange(turretA);
        setExtendedPwmRange(turretB);
    }

    private void setExtendedPwmRange(Servo servo) {
        if (servo.getController() instanceof ServoControllerEx) {
            ServoControllerEx controller = (ServoControllerEx) servo.getController();
            int port = servo.getPortNumber();
            controller.setServoPwmRange(port, new PwmControl.PwmRange(500, 2500));
        }
    }

    // ========================================================================
    // MAIN UPDATE LOOP
    // ========================================================================

    public void update(Follower follower) {
        aimAtFieldPoint(follower.pose(), GOAL_X, GOAL_Y);
    }

    /**
     * Aims at a stationary field point. MovingShotCalculator calls aimAtFieldAngle() instead
     * so that the same servo calibration is used for velocity-compensated shots.
     */
    public void aimAtFieldPoint(Pose robotPose, double goalX, double goalY) {
        double turretX = getTurretX(robotPose);
        double turretY = getTurretY(robotPose);

        double deltaX = goalX - turretX;
        double deltaY = goalY - turretY;

        aimAtFieldAngle(robotPose, Math.atan2(deltaY, deltaX), Math.hypot(deltaX, deltaY));
    }

    /**
     * Aims at an absolute, field-frame bearing. This is intentionally separate from the
     * point-based helper so a moving shot can lead the target without changing servo math.
     */
    public void aimAtFieldAngle(Pose robotPose, double fieldAngle, double distance) {
        currentRobotPose = robotPose;
        distanceToGoal = distance;
        double relativeAngle = normalizeAngle(fieldAngle - robotPose.heading());

        // Apply heading offset in angle space before converting to servo position
        relativeAngle = normalizeAngle(relativeAngle + HEADING_OFFSET);

        double servoRadiansTotal = Math.toRadians(SERVO_DEGREES);

        double servoPosition = 0.5 + (-relativeAngle / servoRadiansTotal) + SERVO_TRIM;

        servoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPosition));

        targetServoPosition = servoPosition;
        setPosition(servoPosition);
    }

    /** Returns the turret pivot's field X coordinate using the existing robot offset convention. */
    public double getTurretX(Pose robotPose) {
        return robotPose.x() - TURRET_OFFSET_FROM_ROBOT_CENTER * Math.cos(robotPose.heading());
    }

    /** Returns the turret pivot's field Y coordinate using the existing robot offset convention. */
    public double getTurretY(Pose robotPose) {
        return robotPose.y() - TURRET_OFFSET_FROM_ROBOT_CENTER * Math.sin(robotPose.heading());
    }

    // ========================================================================
    // CONTROL
    // ========================================================================

    public void setPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        turretA.setPosition(position - 0.01);
        turretB.setPosition(position - 0.01);
    }

    public void adjustTrim(double delta) {
        SERVO_TRIM += delta;
    }

    public void adjustHeadingOffset(double delta) {
        HEADING_OFFSET += delta;
    }

    public void resetHeadingOffset() {
        HEADING_OFFSET = 0.0;
    }

    // ========================================================================
    // HELPERS
    // ========================================================================

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ========================================================================
    // GETTERS FOR TELEMETRY
    // ========================================================================

    public double getPosition()             { return targetServoPosition; }
    public double getDistanceToGoal()       { return distanceToGoal; }
    public double getTargetServoPosition()  { return targetServoPosition; }
    public Pose   getCurrentRobotPose()     { return currentRobotPose; }

    public double getTurretAngleDegrees() {
        return (targetServoPosition - 0.5) * SERVO_DEGREES;
    }
}
