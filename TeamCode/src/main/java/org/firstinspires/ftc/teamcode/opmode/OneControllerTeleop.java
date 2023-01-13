package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name="Full Teleop (One Controller)", group="Iterative Opmode")
public class OneControllerTeleop extends OpMode {

    public static double DRIVE_SPEED_MULTIPLIER = 0.3;
    public static double TURN_SPEED_MULTIPLIER = 0.5;
//    public static double RAISE_ARM_MULTIPLIER = 0.5;

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

//        armPercentage += gamepad1.right_trigger * RAISE_ARM_MULTIPLIER;
//        armPercentage -= gamepad1.left_trigger * RAISE_ARM_MULTIPLIER;
//        armPercentage = Range.clip(armPercentage, 0, 100);
//        telemetry.addData("Arm height: ", armPercentage);
//        telemetry.update();
//
//        robot.setArmHeight(armPercentage);
        robot.armPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
