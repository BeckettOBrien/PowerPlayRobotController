package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Basic Drive", group="Iterative Opmode")
public class BasicDrive extends OpMode {

    private RobotHardware robot;

    public static double DRIVE_SPEED_MULTIPLIER = 0.35;
    public static double TURN_SPEED_MULTIPLIER = 0.5;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        robot.drive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, (gamepad1.x ? 1 : DRIVE_SPEED_MULTIPLIER));
    }
}
