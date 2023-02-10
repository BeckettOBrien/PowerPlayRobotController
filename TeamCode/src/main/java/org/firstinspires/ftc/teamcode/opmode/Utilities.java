package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.vision.SignalSleeveDetector;

@TeleOp(name="Util", group="Iterative Opmode")
@Config
public class Utilities extends OpMode {

    private RobotHardware robot;
    private SignalSleeveDetector detector;

    public static double DRIVE_SPEED_MULTIPLIER = 0.3;
    public static double TURN_SPEED_MULTIPLIER = 0.5;

    public static double ARM_SERVO_POSITION = 0.0;
    public static double CLAW_SERVO_POSITION = 0.0;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        detector = new SignalSleeveDetector(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        sendInfo();
    }

    @Override
    public void loop() {
        double drive = gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0;
        double strafe = gamepad1.dpad_right ? 1 : gamepad1.dpad_left ? -1 : 0;
        double rotate = gamepad1.right_stick_x;

        robot.drive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, (gamepad1.x ? 1 : DRIVE_SPEED_MULTIPLIER));
        robot.armPower(gamepad1.right_trigger - gamepad1.left_trigger);

        robot.rotateArm(ARM_SERVO_POSITION);
        robot.clawGrab.setPosition(CLAW_SERVO_POSITION);

        sendInfo();
    }

    void sendInfo() {
        int raisePos = robot.armLift.getCurrentPosition();
        telemetry.addData("Signal Sleeve", detector.getDeterminedZone());
        telemetry.addData("Arm Lift Encoder", raisePos);
        telemetry.addData("Claw Servo", robot.clawGrab.getPosition());
        telemetry.addData("Arm Servo", robot.armRotate.getPosition());
        telemetry.addData("Left Encoder", robot.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", robot.rightEncoder.getCurrentPosition());
        telemetry.addData("Back Encoder", robot.backEncoder.getCurrentPosition());
        telemetry.update();
    }
}
