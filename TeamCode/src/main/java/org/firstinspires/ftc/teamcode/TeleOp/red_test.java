package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.*;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.*;

import static java.lang.Math.round;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto_cal.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.auto_cal.Turret_Tracking;
import org.firstinspires.ftc.teamcode.sub_const.servo_pos_const;
import org.firstinspires.ftc.teamcode.sub_const.shooter_const;

@TeleOp(name = "decode 23020_RED", group = "2025-2026 Test OP")
public class red_test extends LinearOpMode {

    private TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor; //메카넘
    private DcMotor eat, SA;
    private DcMotorEx SL, SR;
    private Servo servo_S, servo_hood;
    private IMU imu;
    Turret_Tracking tracking = new Turret_Tracking();
    private Follower follower;
    private final Pose startPose = new Pose(72,72,Math.toRadians(90));

    private PIDFController controller;
    private double target_tick;

    private PIDFCoefficients pidfCoefficients;
    private double motor_power;
    private int finalTurretAngle;

    private double targetMotorVelocity;

    //GoBildaPinpointDriver odo;


    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        pidfCoefficients = new PIDFCoefficients(shooter_p, shooter_i, shooter_d, shooter_f);
        controller = new PIDFController(pidfCoefficients);


        FrontLeftMotor  = hardwareMap.dcMotor.get("FL");
        FrontRightMotor = hardwareMap.dcMotor.get("FR");
        BackLeftMotor   = hardwareMap.dcMotor.get("BL");
        BackRightMotor  = hardwareMap.dcMotor.get("BR");

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");
        SR  = hardwareMap.get(DcMotorEx.class, "SR");
        SA = hardwareMap.dcMotor.get("SA");

        /*odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-12, -5);  //cm?
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);*/

        eat.setDirection(DcMotorSimple.Direction.FORWARD);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);


        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        eat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eat.setPower(0);


        servo_S = hardwareMap.servo.get("servo_S");
        servo_S.setPosition(servo_pos_const.servo_shoot_block); // 기본 위치

        servo_hood = hardwareMap.servo.get("servo_H");
        servo_hood.setPosition(servo_pos_const.servo_hood_min);  //기본위치 찾기


        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients
                = new com.qualcomm.robotcore.hardware
                .PIDFCoefficients(flywheel_p, flywheel_i, flywheel_d, flywheel_f);

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);
        SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

        waitForStart();

        while (opModeIsActive()) { //main loop

            follower.update(); //current robot pose update
            Pose current_robot_pos = follower.getPose();  //save to Pose
            Vector current_robot_vel = follower.getVelocity();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) {
                //imu.resetYaw();
                //odo.recalibrateIMU();
            }

            double botHeading_pin = follower.getHeading();

            double rotX = x * Math.cos(-botHeading_pin) - y * Math.sin(-botHeading_pin);
            double rotY = x * Math.sin(-botHeading_pin) + y * Math.cos(-botHeading_pin);

            rotX *= 1.1;

            double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
            );

            FrontLeftMotor.setPower((rotY + rotX - rx) / denominator * slow);
            BackLeftMotor.setPower((rotY - rotX - rx) / denominator * slow);
            FrontRightMotor.setPower((rotY - rotX + rx) / denominator * slow);
            BackRightMotor.setPower((rotY + rotX + rx) / denominator * slow);
            //메카넘 끝






            if (gamepad1.left_bumper) {
                servo_S.setPosition(servo_pos_const.servo_shoot_go);
                eat.setPower(1);
            } else {
                servo_S.setPosition(servo_pos_const.servo_shoot_block);
                eat.setPower(0);
            }

            if (gamepad1.x) eat.setPower(1);

            if (gamepad1.y) eat.setPower(0);

            /*if (gamepad1.left_bumper) {
                servo_S.setPosition(0.35);
                eat.setPower(1);
            } else {
                servo_S.setPosition(0.5);
                eat.setPower(0);
            }*/






            shooter.ShotResult result = shooter.calculateShot(current_robot_pos, RED_GOAL, SCORE_HEIGHT, current_robot_vel, SCORE_ANGLE);

            if (result != null) {

                double StaticTargetPosTicks = tracking.fix_to_goal_RED(current_robot_pos);

                double offsetTicks = (result.turretOffset / (2 * Math.PI)) * SHOOTER_ANGLE_TPR * (105.0/25.0);

                finalTurretAngle = (int) round(StaticTargetPosTicks + offsetTicks);

                double clampedAngle = Range.clip(result.hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
                double hood_servo_pos = mapAngleToServo(clampedAngle);

                servo_hood.setPosition(hood_servo_pos);

                //터렛 pid 계산
                controller.setTargetPosition(finalTurretAngle);

                double currentPos = SA.getCurrentPosition();
                controller.updatePosition(currentPos);

                motor_power = controller.run();
                SA.setPower(motor_power);


            }

            if (gamepad1.a && result != null) {
                targetMotorVelocity = velocityToTicks(result.launchSpeed);

                SL.setVelocity(targetMotorVelocity);
                SR.setVelocity(targetMotorVelocity);
            } else {
                SL.setVelocity(0);
                SR.setVelocity(0);
            }



            ptelemetry.addData("eat Power", eat.getPower());
            ptelemetry.addData("SL Power", SL.getPower());
            ptelemetry.addData("SR Power", SR.getPower());
            ptelemetry.addData("Servo_S Pos", servo_S.getPosition());
            /*telemetry.addData("Heading (deg)",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));*/
            ptelemetry.addData("target encoder", finalTurretAngle);
            ptelemetry.addData("current encoder", SA.getCurrentPosition());
            ptelemetry.addData("curVelo", SL.getVelocity());
            ptelemetry.addData("tarVelo", targetMotorVelocity);
            ptelemetry.update(telemetry);
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
        return revsPerSec * FLYWHEEL_TPR;
    }
}
