package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.TICKS_PER_REV_SHOOTER;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.WHEEL_RADIUS;

import static java.lang.Math.round;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto_cal.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto_cal.Turret_Tracking;

@TeleOp(name = "decode 23020", group = "2024-2025 Test OP")
public class blue_test extends LinearOpMode {

    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;
    private DcMotor eat, SL, SR, SA;
    private Servo servo_S, servo_hood;
    private IMU imu;
    Turret_Tracking tracking = new Turret_Tracking();
    private Follower follower;
    private final Pose startPose = new Pose(72,72,90);


    @Override
    public void runOpMode() throws InterruptedException {

        Pose center = new Pose(72, 72, 90);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);


        FrontLeftMotor  = hardwareMap.dcMotor.get("FL");
        FrontRightMotor = hardwareMap.dcMotor.get("FR");
        BackLeftMotor   = hardwareMap.dcMotor.get("BL");
        BackRightMotor  = hardwareMap.dcMotor.get("BR");

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.dcMotor.get("SL");
        SR  = hardwareMap.dcMotor.get("SR");
        SA = hardwareMap.dcMotor.get("SA");

        eat.setDirection(DcMotorSimple.Direction.FORWARD);
        SL.setDirection(DcMotorSimple.Direction.FORWARD);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eat.setPower(0);
        SL.setPower(0);
        SR.setPower(0);


        servo_S = hardwareMap.servo.get("servo_S");
        servo_S.setPosition(0.5); // 기본 위치

        servo_hood = hardwareMap.servo.get("servo_H");
        servo_hood.setPosition(0.5);  //기본위치 찾기

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

       follower.setPose(center);

        waitForStart();

        while (opModeIsActive()) {

            follower.update(); //current robot pose update
            Pose current_robot_pos = follower.getPose();  //save to Pose
            Vector current_robot_vel = follower.getVelocity();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
            );

            FrontLeftMotor.setPower((rotY + rotX - rx) / denominator * slow);
            BackLeftMotor.setPower((rotY - rotX - rx) / denominator * slow);
            FrontRightMotor.setPower((rotY - rotX + rx) / denominator * slow);
            BackRightMotor.setPower((rotY + rotX + rx) / denominator * slow);
            //메카넘 끝


            servo_S.setPosition(gamepad1.left_bumper ? 0.35 : 0.5);


            if (gamepad1.aWasPressed()) {
                eat.setPower(1);
            }

            if (gamepad1.bWasPressed()) {
                eat.setPower(0);
            }

            if (gamepad1.xWasPressed()) {
                SL.setPower(1);
                SR.setPower(1);
            }

            if (gamepad1.yWasPressed()) {
                SL.setPower(0);
                SR.setPower(0);
            }

            shooter.ShotResult result = shooter.calculateShot(current_robot_pos, BLUE_GOAL, SCORE_HEIGHT, current_robot_vel, SCORE_ANGLE);

            if (check_shooting_zone(current_robot_pos) && result != null) {

                double StaticTargetPosTicks = tracking.fix_to_goal_BLUE(current_robot_pos);

                double offsetTicks = (result.turretOffset / (2 * Math.PI)) * TICKS_PER_REV_SHOOTER * (105.0/25.0);

                int finalTurretAngle = (int) round(StaticTargetPosTicks + offsetTicks);

                SA.setTargetPosition(finalTurretAngle);
                SA.setPower(0.3); //pid 제어하면 없어질 것
                SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);  //heading to goal

                double clampedAngle = Range.clip(result.hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
                double hood_servo_pos = mapAngleToServo(clampedAngle);

                servo_hood.setPosition(hood_servo_pos);

                double targetMotorVelocity = velocityToTicks(result.launchSpeed);

                ((com.qualcomm.robotcore.hardware.DcMotorEx) SL).setVelocity(targetMotorVelocity);
                ((com.qualcomm.robotcore.hardware.DcMotorEx) SR).setVelocity(targetMotorVelocity);
            } else {
                SL.setPower(0);
                SR.setPower(0);
            }

            telemetry.addData("eat Power", eat.getPower());
            telemetry.addData("SL Power", SL.getPower());
            telemetry.addData("SR Power", SR.getPower());
            telemetry.addData("Servo_S Pos", servo_S.getPosition());
            telemetry.addData("Heading (deg)",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("target encoder", SA.getTargetPosition());
            telemetry.addData("current encoder", SA.getCurrentPosition());
            telemetry.update();
        }
    }

    public boolean check_shooting_zone(Pose pose) {
        if (pose.getY() >= Math.abs(pose.getX() - 72) + 72) return true;  // Y >= |x-72| + 72
        if (pose.getY() <= -Math.abs(pose.getX() - 72) + 72) return true; // Y <= -|x-72| + 72
        return false;
    }

    // 선형 보간 함수 (Linear Interpolation)
    private double mapAngleToServo(double angleRad) {
        double slope = (HOOD_SERVO_MIN - HOOD_SERVO_MAX) / (HOOD_MIN_ANGLE - HOOD_MAX_ANGLE);
        return slope * (angleRad - HOOD_MIN_ANGLE) + HOOD_SERVO_MIN;
    }

    // 속도(in/s)를 모터 속도(Ticks/s)로 변환
    private double velocityToTicks(double velocityInchesPerSec) {
        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS;
        double revsPerSec = velocityInchesPerSec / wheelCircumference;
        return revsPerSec * TICKS_PER_REV_SHOOTER;
    }
}
