package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.sub_const.pos_const.*;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.*;


import static java.lang.Math.round;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto_cal.Turret_Tracking;
import org.firstinspires.ftc.teamcode.auto_cal.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.sub_const.servo_pos_const;

@Autonomous(name = "red_test_far", group = "2025-2026 Test_red", preselectTeleOp = "red_test")
public class red_test extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private DcMotor eat, SA;
    private DcMotorEx SL, SR;
    private Servo servo_S, servo_hood;
    Turret_Tracking tracking = new Turret_Tracking();
    private PIDFController controller;
    private PIDFCoefficients pidfCoefficients;
    private double motor_power;
    private int finalTurretAngle;
    private double targetMotorVelocity;



    private Path go1_path, go2_path;
    private PathChain eat_shoot1, eat_shoot2, eat_shoot3, eat_shoot4;



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(RED_CLOSE_START);

        //////////////////////////////////////////

        pidfCoefficients = new PIDFCoefficients(shooter_p, shooter_i, shooter_d, shooter_f);
        controller = new PIDFController(pidfCoefficients);

        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");
        SR  = hardwareMap.get(DcMotorEx.class, "SR");
        SA = hardwareMap.dcMotor.get("SA");

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

    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void loop() {
        follower.update();
        Pose current_robot_pos = follower.getPose();  //save to Pose
        Vector current_robot_vel = follower.getVelocity();

        autonomousPathUpdate();


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

            double targetMotorVelocity = velocityToTicks(result.launchSpeed);
        }








        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("path", pathState);

        panelsTelemetry.update(telemetry);

    }


    public void buildPaths() { //경로 만들기

//        go1_path = new Path(new BezierLine(startPose, middlePose));
//        go1_path.setLinearHeadingInterpolation(startPose.getHeading(), middlePose.getHeading());


        eat_shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(RED_CLOSE_START,
                        new Pose(79, 80, Math.toRadians(0)),
                        RED_CLOSE_EAT1))
                .setLinearHeadingInterpolation(RED_CLOSE_START.getHeading(), RED_CLOSE_EAT1.getHeading())

                .addPath(new BezierLine(RED_CLOSE_EAT1, RED_CLOSE_SHOOT1))
                .setLinearHeadingInterpolation(RED_CLOSE_EAT1.getHeading(), RED_CLOSE_SHOOT1.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {  //경로 상태 관리하기
        switch (pathState) {
            case 0:
                follower.followPath(eat_shoot1);

                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(go2_path);

                    SL.setVelocity(targetMotorVelocity);
                    SR.setVelocity(targetMotorVelocity);

                    setPathState(2);
                }
                break;

//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(chain_test);
//                    setPathState(-1);
//                }
//                break;
        }
    }

    public void setPathState(int pState) {  //경로상태 업데이트
        pathState = pState;
        pathTimer.resetTimer();
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

