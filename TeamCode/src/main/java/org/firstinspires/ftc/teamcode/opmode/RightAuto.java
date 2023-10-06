package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.vision.SignalSleeveDetector;

@Autonomous
@Config
public class RightAuto extends LinearOpMode {

    RobotHardware robot;
    SampleMecanumDrive drive;
    SignalSleeveDetector detector;

    enum AUTO_STATE {
        WAIT,
        TO_FIRST_DROP,
        TO_FIRST_PICKUP,
        TO_DROP,
        TO_PICKUP
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        robot = new RobotHardware(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = drive.robot;
        detector = new SignalSleeveDetector(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();

        drive.setPoseEstimate(new Pose2d(-35, 60, Math.toRadians(-90)));
        TrajectorySequence toFirstDrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-35, 34))
                .strafeTo(new Vector2d(-12, 34))
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence toFirstPickup = drive.trajectorySequenceBuilder(toFirstDrop.end())
                .lineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(0)))
                .back(40)
                .build();

        TrajectorySequence toDrop = drive.trajectorySequenceBuilder(toFirstPickup.end())
                .forward(40)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence toPickup = drive.trajectorySequenceBuilder(toDrop.end())
                .turn(Math.toRadians(-45))
                .back(40)
                .build();
//                                    .forward(40)
//                                    .turn(Math.toRadians(45))
//                                    // drop
//                                    .turn(Math.toRadians(-45))
//                                    // repeat
//                                    .lineTo(new Vector2d(-20, 12))
//                                    .build();

        AUTO_STATE currentState = AUTO_STATE.WAIT;
//        AUTO_STATE nextState = AUTO_STATE.TO_FIRST_DROP;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("State", currentState);
            switch (currentState) {
                case WAIT:
                    if (gamepad1.a) {
                        currentState = AUTO_STATE.TO_FIRST_DROP;
                        robot.grabClaw();
                        robot.setLift(0.6);
                        sleep(2500);
                        robot.setLift(0);
//                        sleep(1000);
//                        robot.setLift(0);
                        drive.followTrajectorySequence(toFirstDrop);
                        robot.releaseClaw();
                    }
                    break;
                case TO_FIRST_DROP:
                    if (!drive.isBusy() && gamepad1.a) {
                        currentState = AUTO_STATE.TO_FIRST_PICKUP;
                        drive.followTrajectorySequence(toFirstPickup);
                    }
                    break;
                case TO_FIRST_PICKUP:
                    if (!drive.isBusy() && gamepad1.a) {
                        currentState = AUTO_STATE.TO_DROP;
                        drive.followTrajectorySequence(toDrop);
                    }
                    break;
                case TO_DROP:
                    if (!drive.isBusy() && gamepad1.a) {
                        currentState = AUTO_STATE.TO_PICKUP;
                        drive.followTrajectorySequence(toPickup);
                    }
                    break;
                case TO_PICKUP:
                    if (!drive.isBusy() && gamepad1.a) {
                        currentState = AUTO_STATE.TO_DROP;
                        drive.followTrajectorySequence(toDrop);
                    }
                    break;
            }
        }



//        robot.brake();
    }
}
