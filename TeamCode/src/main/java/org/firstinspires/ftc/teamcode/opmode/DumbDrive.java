package org.firstinspires.ftc.teamcode.opmode;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.vision.SignalSleeveDetector;

@Autonomous
@Config
public class DumbDrive extends LinearOpMode {

    public static double DRIVE_SPEED = 0.2;
    public static int SLEEP_TIME = 750;

    RobotHardware robot;
    SignalSleeveDetector cc;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        cc = new SignalSleeveDetector(hardwareMap);

        SignalSleeveDetector.PARK_ZONE a = cc.getDeterminedZone();
        waitForStart();

        robot.grabClaw();
        robot.sleep(500);

        switch(a)
        {
            case RIGHT: {
                robot.drive(0,1,0, DRIVE_SPEED);
            }
            case LEFT:
            {
                robot.drive(0,-1,0, DRIVE_SPEED);
            }
            case MIDDLE:{
                robot.drive(1, 0, 0, DRIVE_SPEED);
            }
        }

        robot.sleep(SLEEP_TIME);
        robot.brake();
    }
}
