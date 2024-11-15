package org.firstinspires.ftc.teamcode;

import android.opengl.GLDebugHelper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name ="into_the_deep-testcode", group = "Into_the_deep")


public class CRCLTEST extends LinearOpMode {


    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor rightFront = hardwareMap.dcMotor.get("FR");
        DcMotor leftFront = hardwareMap.dcMotor.get("FL");
        DcMotor leftRear = hardwareMap.dcMotor.get("BL");
        DcMotor rightRear = hardwareMap.dcMotor.get("BR");
        DcMotor centerleft = hardwareMap.dcMotor.get("CL");
        DcMotor centerright = hardwareMap.dcMotor.get("CR");
        Servo BWL = hardwareMap.servo.get("BWL"); //Bucket Wrist Left Servo
        Servo BWR = hardwareMap.servo.get("BWR"); //Bucket Wrist right Servo
        Servo BG = hardwareMap.servo.get("BG"); // Bucket Gripper Servo
        Servo GG = hardwareMap.servo.get("GG"); // Ground Gripper Servo
        Servo SL = hardwareMap.servo.get("SL"); //Slide Left Servo
        Servo SR = hardwareMap.servo.get("SR"); //Slide right Servo
        Servo GGL = hardwareMap.servo.get("GGL"); // Ground Gripper Left Servo
        Servo GGR = hardwareMap.servo.get("GGR"); // Ground Gripper right Servo
        Servo WL = hardwareMap.servo.get("WL"); // Wrist Left Servo
        Servo WR = hardwareMap.servo.get("WR"); // Wrist right Servo

        boolean swUpstatus = false;

        boolean swUpcurrent;

        boolean swDownstatus = false;

        boolean swDowncurrent;


        int targetPosition = 0;

        int currentPosition = 0;

        double wPOSITION = 0;

        swDowncurrent = gamepad2.dpad_left;

        swUpcurrent = gamepad2.dpad_right;

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            telemetry.addData("encoder", centerleft.getCurrentPosition());

            telemetry.addData("wPosition", wPOSITION);

            telemetry.addData("X", gamepad1.left_stick_x);

            telemetry.addData("BG", BG);

            telemetry.addData("GG", GG);

            telemetry.addData("SR", SR);

            telemetry.addData("SL", SL);

            telemetry.addData("GGL", GGL);

            telemetry.addData("GGR", GGR);

            telemetry.update();


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
            BWL.setDirection(Servo.Direction.REVERSE);
            SL.setDirection(Servo.Direction.REVERSE);
            GGL.setDirection(Servo.Direction.REVERSE);
            WL.setDirection(Servo.Direction.REVERSE);


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double S = gamepad2.left_stick_y;
            double K = gamepad2.left_stick_y;
            double D = gamepad2.right_stick_y;
            double J = gamepad2.right_stick_y;
            double F = gamepad1.right_stick_y;
            double H = gamepad1.right_stick_y;

            double slow = 1.5 - (0.7 * gamepad1.right_trigger);

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
            double centerleftPower = S * slow;
            double centerrightPower = K * slow;
            double SlideLeftPower = D * slow;
            double SliderightPower = J * slow;
            double WristLeftPower = F * slow;
            double WristRightPower = H * slow;



            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
            centerleft.setPower(centerleftPower);
            centerright.setPower(centerrightPower);
            WristLeftPower = F * slow;
            if (gamepad2.left_bumper) { // 장빈 버킷 집게
                BG.setPosition(0.6);
            } else {
                BG.setPosition(0.95);
            }

            if (gamepad1.right_bumper) { //대호 아래 집게
                GG.setPosition(0.6);
            } else {
                GG.setPosition(0.95);
            }


        }
        if (gamepad2.dpad_down) {
            targetPosition = 1340; // 장빈 1번째 버킷
            centerleft.setTargetPosition(targetPosition);
            centerright.setTargetPosition(targetPosition);
            centerright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            centerleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            centerleft.setPower(1);
            centerright.setPower(1);
            currentPosition = centerleft.getCurrentPosition();
            currentPosition = centerright.getCurrentPosition();
            BWR.setPosition(0.65); // 바스켓에 넣기 좋은 각도
            BWL.setPosition(0.65); // 바스켓에 넣기 좋은 각도

        }
        if (gamepad2.dpad_up) {
            targetPosition = 3340; // 장빈 2번째 버킷
            centerleft.setTargetPosition(targetPosition);
            centerright.setTargetPosition(targetPosition);
            centerright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            centerleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            centerleft.setPower(1);
            centerright.setPower(1);
            currentPosition = centerleft.getCurrentPosition();
            currentPosition = centerright.getCurrentPosition();
            BWR.setPosition(0.65); // 바스켓에 넣기 좋은 각도
            BWL.setPosition(0.65); // 바스켓에 넣기 좋은 각도
        }




        }
    }




    // 대호 차체를 움직임 밑 손목 gamepad1
    // 장빈 팔목 손목 아래 팔 다 장빈 버켓 그리퍼 옮길때 손목값 gamepad2
//// centerleftPower = 1340; 둘기둥 1
// centerrightPower = 1340;

//it git right?
//y키 누르면 그리퍼 잡는거(패스) ㄱㄱ 할거임 어렵다