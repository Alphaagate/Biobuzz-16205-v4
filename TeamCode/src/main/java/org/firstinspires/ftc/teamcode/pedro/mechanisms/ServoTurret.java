package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoTurret {   // name kept so Robot/MovingShotCalculator still compile

    private DcMotorEx turretMotor;

    // FIELD
    public static double GOAL_X = 60.0;
    public static double GOAL_Y = 60.0;

    // PHYSICAL
    public static double TURRET_OFFSET_FROM_ROBOT_CENTER = 1.0;

    // MOTOR / GEARING  (CHANGE THESE FOR YOUR ROBOT)
    public static double MOTOR_TICKS_PER_REV = 383.6;   // 435 RPM (13.7:1) Yellow Jacket
    public static double GEAR_RATIO = 80.0 / 27.0;      // 2.9630

    // SOFT LIMITS (degrees, 0 = straight forward, + = counterclockwise)
    public static double MIN_ANGLE_DEG = -170.0;
    public static double MAX_ANGLE_DEG = 170.0;

    // TUNING
    public static double MAX_POWER = 0.6;
    public static double POSITION_P = 10.0;           // RUN_TO_POSITION P gain
    public static int TOLERANCE_TICKS = 8;
    public static double TRIM_DEG = 0.0;
    public static double HEADING_OFFSET = 0.0;        // radians
    public static double HEADING_OFFSET_STEP = Math.toRadians(2.0);

    // STATE
    private double distanceToGoal = 0;
    private int targetTicks = 0;
    private double targetAngleDeg = 0;
    private Pose currentRobotPose = null;

    public void init(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD); // flip if + ticks isn't CCW
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPositionPIDFCoefficients(POSITION_P);
        turretMotor.setPower(MAX_POWER);
        // IMPORTANT: turret must be physically centered (facing forward) at init.
    }

    public void update(Follower follower) {
        aimAtFieldPoint(follower.pose(), GOAL_X, GOAL_Y);
    }

    public void aimAtFieldPoint(Pose robotPose, double goalX, double goalY) {
        double dx = goalX - getTurretX(robotPose);
        double dy = goalY - getTurretY(robotPose);
        aimAtFieldAngle(robotPose, Math.atan2(dy, dx), Math.hypot(dx, dy));
    }

    public void aimAtFieldAngle(Pose robotPose, double fieldAngle, double distance) {
        currentRobotPose = robotPose;
        distanceToGoal = distance;

        double relative = normalizeAngle(fieldAngle - robotPose.heading());
        relative = normalizeAngle(relative + HEADING_OFFSET);

        double deg = Math.toDegrees(relative) + TRIM_DEG;
        deg = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, deg)); // protect cables

        targetAngleDeg = deg;
        targetTicks = (int) Math.round(deg * ticksPerDegree());

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(MAX_POWER);
    }

    public double getTurretX(Pose p) {
        return p.x() - TURRET_OFFSET_FROM_ROBOT_CENTER * Math.cos(p.heading());
    }
    public double getTurretY(Pose p) {
        return p.y() - TURRET_OFFSET_FROM_ROBOT_CENTER * Math.sin(p.heading());
    }

    private double ticksPerDegree() {
        return MOTOR_TICKS_PER_REV * GEAR_RATIO / 360.0;
    }

    public void stop() { turretMotor.setPower(0); }

    public boolean atTarget() {
        return Math.abs(turretMotor.getCurrentPosition() - targetTicks) <= TOLERANCE_TICKS;
    }

    public void adjustTrim(double deltaDeg) { TRIM_DEG += deltaDeg; }
    public void adjustHeadingOffset(double d) { HEADING_OFFSET += d; }
    public void resetHeadingOffset() { HEADING_OFFSET = 0.0; }

    private double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // Telemetry
    public double getDistanceToGoal() { return distanceToGoal; }
    public int getTargetTicks() { return targetTicks; }
    public double getTargetAngleDegrees() { return targetAngleDeg; }
    public double getTurretAngleDegrees() { return turretMotor.getCurrentPosition() / ticksPerDegree(); }
    public Pose getCurrentRobotPose() { return currentRobotPose; }
}