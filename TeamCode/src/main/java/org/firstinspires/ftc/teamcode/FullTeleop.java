package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Full Teleop", group="Iterative Opmode")
public class FullTeleop extends OpMode {

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
        robot.armLiftPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if (gamepad1.a) {
            robot.grabClaw();
        }
        if (gamepad1.b) {
            robot.releaseClaw();
        }

        if (gamepad1.left_bumper) {
            robot.rotateArm(RobotHardware.ARM_FORWARD);
        }
        if (gamepad1.right_bumper) {
            robot.rotateArm(RobotHardware.ARM_BACKWARD);
        }
    }
}
