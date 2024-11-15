package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.opengl.AutoConfigGLSurfaceView;


@TeleOp


public class TestCode extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor rightFront = hardwareMap.dcMotor.get("FR");
        DcMotor leftFront = hardwareMap.dcMotor.get("FL");
        DcMotor leftRear = hardwareMap.dcMotor.get("BL");
        DcMotor rightRear = hardwareMap.dcMotor.get("BR");
        Servo GGL = hardwareMap.servo.get("GGL"); // Ground Gripper Left Servo
        Servo GGR = hardwareMap.servo.get("GGR"); // Ground Gripper right Servo
        Servo WR = hardwareMap.servo.get("WR"); // 팔전체 돌리는 서보
        Servo SR = hardwareMap.servo.get("SR"); // 팔 늘어나는 서보


        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        imu.initialize(parameters);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        GGL.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("GGR", GGR.getPosition());
            telemetry.addData("GGR",GGR);

            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double slow = 1.8 - (0.9 * gamepad1.right_trigger);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = X * Math.cos(-botHeading) - Y * Math.sin(-botHeading);
            double rotY = X * Math.sin(-botHeading) + Y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(R), 1);
            double leftFrontPower = ((rotY + rotX + R) / denominator) * slow;
            double leftRearPower = ((rotY - rotX + R) / denominator) * slow;
            double rightFrontPower = ((rotY - rotX - R) / denominator) * slow;
            double rightRearPower = ((rotY + rotX - R) / denominator) * slow;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
//GGR 0.5보다 크면 올라감. GGL 0.5보다 크면 올라감. WR 0.5보다 작아지면 올라감. SR 커지면 길어짐.

            if (gamepad1.y) {
                SR.setPosition(0.8);
            }
            if (gamepad1.a) {
                SR.setPosition(0.45);
            }

            if (gamepad1.x) {
                GGR.setPosition(0.5);
            }
            if (gamepad1.dpad_up) {
                GGR.setPosition(0.5);
                GGL.setPosition(0.5);
                WR.setPosition(0.05);
            }
            if (gamepad1.dpad_down) {
                GGR.setPosition(0.15);
                GGL.setPosition(0.15);
                WR.setPosition(0.45);
            }
            if (gamepad1.dpad_right) {
                GGL.setPosition(0.5);
            }

        }

    }
}