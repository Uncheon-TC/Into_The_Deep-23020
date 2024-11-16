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


@TeleOp


public class KLA_24906_V2 extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        DcMotor rightFront = hardwareMap.dcMotor.get("FR");
        DcMotor leftFront = hardwareMap.dcMotor.get("FL");
        DcMotor leftRear = hardwareMap.dcMotor.get("BL");
        DcMotor rightRear = hardwareMap.dcMotor.get("BR");
        DcMotor centerleft = hardwareMap.dcMotor.get("CL");
        DcMotor centerright = hardwareMap.dcMotor.get("CR");
        Servo BWR = hardwareMap.servo.get("BWR"); //Bucket Wrist right Servo
        Servo SR = hardwareMap.servo.get("SR"); //Slide right Servo
        Servo GGL = hardwareMap.servo.get("GGL"); // Ground Gripper Left Servo
        Servo GGR = hardwareMap.servo.get("GGR"); // Ground Gripper right Servo
        Servo WR = hardwareMap.servo.get("WR"); // Wrist right Servo
        Servo GG = hardwareMap.servo.get("GG");
        Servo BG = hardwareMap.servo.get("BG");

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
        centerleft.setDirection(DcMotorSimple.Direction.REVERSE);
        GGL.setDirection(Servo.Direction.REVERSE);

        centerleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        boolean swUpstatus = false;
        boolean swUpcurrent;
        boolean swDownstatus = false;
        boolean swDowncurrent;
        boolean bwUpstatus = false;
        boolean bwUpcurrent;
        boolean bwDownstatus = false;
        boolean bwDowncurrent;
        float rtupstatus;
        int targetPosition = 0;

        int currentPosition = 0;
        double wPOSITION = 0;
        double bPosition = 0;
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("GGR", GGR.getPosition());
            telemetry.addData("SR", SR.getPosition());
            telemetry.addData("BWR", BWR.getPosition());
            telemetry.update();
            swDowncurrent = gamepad1.dpad_right;
            swUpcurrent = gamepad1.dpad_left;
            bwDowncurrent = gamepad2.dpad_right;
            bwUpcurrent = gamepad2.dpad_left;

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double D = gamepad2.right_trigger;
            double J = gamepad2.left_trigger;
            double slow = 1.0 - (0.7 * gamepad1.right_trigger);

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
            double bucketwristright = D * slow;
            double bucketwristleft = J * slow;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

//GGR 0.5보다 크면 올라감. GGL 0.5보다 크면 올라감. WR 0.5보다 작아지면 올라감. SR 커지면 길어짐.


            if (gamepad2.dpad_up) {
                targetPosition = 1450;
                centerleft.setTargetPosition(targetPosition);
                centerright.setTargetPosition(targetPosition);
                centerleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                centerright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                centerleft.setPower(0.8);
                centerright.setPower(0.8);

            }

            if (gamepad2.dpad_down) {
                targetPosition = 50;
                centerleft.setTargetPosition(targetPosition);
                centerright.setTargetPosition(targetPosition);
                centerleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                centerright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                centerleft.setPower(0.8);
                centerright.setPower(0.8);
            }


            if (gamepad2.left_bumper) {
                targetPosition = 1150;
                centerleft.setTargetPosition(targetPosition);
                centerright.setTargetPosition(targetPosition);
                centerleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                centerright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                centerleft.setPower(0.8);
                centerright.setPower(0.8);
            }


            if (gamepad2.y) {
                SR.setPosition(0.8);
            }
            if (gamepad2.a) {
                SR.setPosition(0.45);
            }

            if (gamepad2.x) {
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
                WR.setPosition(0.4);
            }
            rtupstatus = gamepad2.right_trigger;

            if (gamepad2.right_trigger != 0) {
                BWR.setPosition(0.85);
            }

            if (gamepad2.left_trigger != 0) {
                BWR.setPosition(0.05);
            }
            if (gamepad1.a)
                WR.setPosition(0.48);

            if (gamepad1.right_bumper) {
                GG.setPosition(0.7);
            }
            else{
                GG.setPosition(1);
            }

            if (gamepad2.b) {
                BG.setPosition(1);
            }
            else{
                BG.setPosition(0.7);
            }




            }
    }
}
