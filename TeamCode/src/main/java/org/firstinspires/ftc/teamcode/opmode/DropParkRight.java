package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.vision.SignalSleeveDetector;

@Autonomous
@Config
public class DropParkRight extends LinearOpMode {

    public static double FORWARD_DISTANCE = 26;
    public static double STRAFE_DISTANCE = 28;

    RobotHardware robot;
    SampleMecanumDrive drive;
    SignalSleeveDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
//        robot = new RobotHardware(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = drive.robot;
        detector = new SignalSleeveDetector(hardwareMap);

        TrajectorySequence toDrop = drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                .forward(26)
                .turn(Math.toRadians(50))
                .forward(11)
                .build();

        TrajectorySequenceBuilder toPark = drive.trajectorySequenceBuilder(toDrop.end())
                .lineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)));

        waitForStart();

        drive.setPoseEstimate(toDrop.start());

//        TrajectorySequenceBuilder

        switch (detector.getDeterminedZone()) {
            case LEFT:
                toPark = toPark.strafeLeft(STRAFE_DISTANCE);
                break;
            case RIGHT:
                toPark = toPark.strafeRight(STRAFE_DISTANCE);
                break;
        }

        robot.grabClaw();
        robot.sleep(500);
//        robot.setLift(0.5);
//        robot.sleep(2500);
//        robot.setLift(0);
        robot.liftTo(2075);
        robot.rotateArm(RobotHardware.ARM_FORWARD);
//        while (robot.armLift.getCurrentPosition() < 2000) sleep(1);
        drive.followTrajectorySequence(toDrop);
        robot.releaseClaw();
        drive.followTrajectorySequence(toPark.build());
        robot.liftTo(50);

//        switch(detector.getDeterminedZone())
//        {
//            case RIGHT: {
//                robot.drive(0,1,0, DRIVE_SPEED);
//                break;
//            }
//            case LEFT:
//            {
//                robot.drive(0,-1,0, DRIVE_SPEED);
//                break;
//            }
//            case MIDDLE:
//                robot.brake();
//                return;
//        }
//
//        robot.sleep(STRAFE_SLEEP_TIME);
//        robot.brake();
    }
}
