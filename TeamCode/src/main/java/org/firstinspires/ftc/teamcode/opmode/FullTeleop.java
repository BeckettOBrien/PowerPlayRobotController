package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name="Full Teleop", group="Iterative Opmode")
public class FullTeleop extends OpMode {

    public static double DRIVE_SPEED_MULTIPLIER = 0.4;
    public static double TURN_SPEED_MULTIPLIER = 0.85;
    public static double RAISE_ARM_MULTIPLIER = 0.55;

    private RobotHardware robot;

    double armPercentage = 0;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : -gamepad1.left_stick_y;
        double strafe = gamepad1.dpad_right ? 1 : gamepad1.dpad_left ? -1 :gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        robot.drive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, (gamepad1.x ? 1 : DRIVE_SPEED_MULTIPLIER));

        if (gamepad2.a) {
            robot.grabClaw();
        }
        if (gamepad2.b) {
            robot.releaseClaw();
        }

        if (gamepad2.left_bumper || gamepad2.dpad_left) {
            robot.rotateArm(RobotHardware.ARM_FORWARD);
        }
        if (gamepad2.right_bumper || gamepad2.dpad_right) {
            robot.rotateArm(RobotHardware.ARM_BACKWARD);
        }

//        armPercentage += gamepad1.right_trigger * RAISE_ARM_MULTIPLIER;
//        armPercentage -= gamepad1.left_trigger * RAISE_ARM_MULTIPLIER;
//        armPercentage = Range.clip(armPercentage, 0, 100);
//        telemetry.addData("Arm height: ", armPercentage);
//        telemetry.update();
//
//        robot.setArmHeight(armPercentage);
        robot.setLift((gamepad2.right_trigger - gamepad2.left_trigger) * RAISE_ARM_MULTIPLIER);
    }
}
