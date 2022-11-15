package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Config
public class DumbDrive extends LinearOpMode {

    public static double DRIVE_SPEED = 0.2;
    public static int SLEEP_TIME = 750;

    RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        waitForStart();

        robot.grabClaw();
        robot.sleep(500);
        robot.drive(1, 0, 0, DRIVE_SPEED);
        robot.sleep(SLEEP_TIME);
        robot.brake();
    }
}
