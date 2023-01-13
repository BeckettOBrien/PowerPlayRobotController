package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@TeleOp(name="Basic Drive", group="Iterative Opmode")
public class BasicDrive extends OpMode {

    private RobotHardware robot;

    public static double DRIVE_SPEED_MULTIPLIER = 0.3;
    public static double TURN_SPEED_MULTIPLIER = 0.5;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0;
        double strafe = gamepad1.dpad_right ? 1 : gamepad1.dpad_left ? -1 : 0;
        double rotate = gamepad1.right_stick_x;

        robot.drive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, (gamepad1.x ? 1 : DRIVE_SPEED_MULTIPLIER));
//        robot.armLiftPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
