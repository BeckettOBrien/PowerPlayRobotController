package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.vision.SignalSleeveDetector;

@Autonomous
@Config
public class OnlyPark extends LinearOpMode {

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

        waitForStart();

        TrajectorySequenceBuilder path = drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .forward(FORWARD_DISTANCE);

        switch (detector.getDeterminedZone()) {
            case LEFT:
                path = path.strafeLeft(STRAFE_DISTANCE);
                break;
            case RIGHT:
                path = path.strafeRight(STRAFE_DISTANCE);
                break;
        }

        robot.grabClaw();
        robot.sleep(500);
        robot.setLift(0.5);
        robot.sleep(1000);
        robot.setLift(0);
        drive.followTrajectorySequence(path.build());

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
