package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.vision.SignalSleeveDetector;

@Autonomous
@Config
public class DumbDrive extends LinearOpMode {

    public static double DRIVE_SPEED = 0.2;
    public static int FORWARD_SLEEP_TIME = 750;
    public static int STRAFE_SLEEP_TIME = 500;

    RobotHardware robot;
    SignalSleeveDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        detector = new SignalSleeveDetector(hardwareMap);

        waitForStart();

        robot.grabClaw();
        robot.sleep(500);
        robot.drive(1, 0, 0, DRIVE_SPEED);
        robot.sleep(FORWARD_SLEEP_TIME);

        switch(detector.getDeterminedZone())
        {
            case RIGHT: {
                robot.drive(0,1,0, DRIVE_SPEED);
                break;
            }
            case LEFT:
            {
                robot.drive(0,-1,0, DRIVE_SPEED);
                break;
            }
            case MIDDLE:
                robot.brake();
                return;
        }

        robot.sleep(STRAFE_SLEEP_TIME);
        robot.brake();
    }
}
