package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AprilTag Cluster Test")
public class AprilTagClusterTest extends AbstractAprilTag {

    @Override
    protected String getAllianceLetter() { return "R"; }   // change to "B" for blue

    @Override
    public void runOpMode() {
        initAprilTag();
        telemetry.addLine("Ready - camera preview: 3 dots > Camera Stream");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            detectAprilTag();
            if (isTargetFound()) {
                telemetry.addData("Bearing (turn error)", getTargetCluster().ftcPose.bearing);
            }
            telemetry.update();
        }
        stopVisionPortal();
    }
}