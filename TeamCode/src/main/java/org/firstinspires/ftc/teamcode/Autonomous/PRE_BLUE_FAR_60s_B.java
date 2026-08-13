package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_FAR_15_EAT2;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_FAR_15_SHOOT;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_FAR_15_START;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_FAR_EAT_1;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_FAR_EAT_2;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_FAR_EAT_2_CP;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.FLYWHEEL_TPR;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SHOOTER_ANGLE_TPR;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_d;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_f;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_i;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.flywheel_p;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_d;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_f;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_i;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.shooter_p;
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
import org.firstinspires.ftc.teamcode.sub_const.pos_const;
import org.firstinspires.ftc.teamcode.sub_const.servo_pos_const;

@Autonomous(name = "AUTO_PRE_BLUE_FAR_60s_B", group = "2026Premiere", preselectTeleOp = "TELEOP_BLUE_Priemier")
public class PRE_BLUE_FAR_60s_B extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private static final double SHOOTER_POWER_RATIO = 0.67; //속도 오프셋
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, segmentTime;
    private int pathState;


    private DcMotor eat, SA;
    private DcMotorEx SL, SR;
    private Servo servo_S, servo_hood, servo_eat;
    Turret_Tracking tracking = new Turret_Tracking();
    private PIDFController controller;
    private PIDFCoefficients pidfCoefficients;
    private double motor_power;
    private int finalTurretAngle;
    private double targetMotorVelocity;
    private boolean segmentStarted = false;

    private double shooter_power = 0;

    private Path FE,RSF,SE,RSS,PARK;

    private PathChain cycle1,cycle2,parking;


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        segmentTime = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(BLUE_FAR_15_START);

        //////////////////////////////////////////

        pidfCoefficients = new PIDFCoefficients(shooter_p, shooter_i, shooter_d, shooter_f);
        controller = new PIDFController(pidfCoefficients);

        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");
        SR  = hardwareMap.get(DcMotorEx.class, "SR");
        SA = hardwareMap.dcMotor.get("SA");

        eat.setDirection(DcMotorSimple.Direction.REVERSE);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);


        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        eat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eat.setPower(0);

        servo_S = hardwareMap.servo.get("servo_S");
        servo_S.setPosition(servo_pos_const.servo_shoot_block); // 기본 위치

        servo_hood = hardwareMap.servo.get("servo_H");
        servo_hood.setPosition(servo_pos_const.servo_hood_min);  //기본위치 찾기

        servo_eat = hardwareMap.servo.get("servo_EAT");
        servo_eat.setPosition(servo_pos_const.servo_eat_down);


        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients
                = new com.qualcomm.robotcore.hardware
                .PIDFCoefficients(flywheel_p, flywheel_i, flywheel_d, flywheel_f);

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);
        //SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void loop() {
        if (opmodeTimer.getElapsedTimeSeconds() >= 60.0) {
            requestOpModeStop();
            return;
        }
        follower.update();

        Pose current_robot_pos = follower.getPose();  //save to Pose
        Vector current_robot_vel = follower.getVelocity();


        shooter.ShotResult result = shooter.calculateShot(current_robot_pos, BLUE_GOAL, SCORE_HEIGHT, current_robot_vel, SCORE_ANGLE);
        if (result != null && pathState < 100) {

            double StaticTargetPosTicks = tracking.fix_to_goal_BLUE(current_robot_pos);

            double offsetTicks = (result.turretOffset / (2 * Math.PI)) * SHOOTER_ANGLE_TPR * (105.0/25.0);

            finalTurretAngle = (int) round(StaticTargetPosTicks - offsetTicks);

            double clampedAngle = Range.clip(result.hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
            double hood_servo_pos = mapAngleToServo(clampedAngle);

            servo_hood.setPosition(hood_servo_pos);

            //터렛 pid 계산
            controller.setTargetPosition(finalTurretAngle);

            double currentPos = SA.getCurrentPosition();
            controller.updatePosition(currentPos);

            motor_power = controller.run();
            SA.setPower(motor_power);

            targetMotorVelocity = velocityToTicks(result.launchSpeed);

            SL.setVelocity(targetMotorVelocity*SHOOTER_POWER_RATIO);
            shooter_power = SL.getPower();
            SR.setPower(shooter_power);
        }

        autonomousPathUpdate();

        pos_const.savedAutoPose = follower.getPose();



        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("path", pathState);

        panelsTelemetry.addData("turret_target", finalTurretAngle);
        panelsTelemetry.addData("turret_current", SA.getCurrentPosition());

        panelsTelemetry.addData("target_velo", targetMotorVelocity);
        panelsTelemetry.addData("current_velo", SL.getVelocity());
        panelsTelemetry.addData("cueent_vele_nonoff", SL.getVelocity()/0.64);

        //panelsTelemetry.addData("velo")

        panelsTelemetry.update(telemetry);

    }


    public void buildPaths() { //경로 만들기

        FE = new Path(new BezierLine(BLUE_FAR_15_START, BLUE_FAR_EAT_1));
        FE.setLinearHeadingInterpolation(BLUE_FAR_15_START.getHeading(), BLUE_FAR_EAT_1.getHeading());

        RSF = new Path(new BezierLine(BLUE_FAR_EAT_1, BLUE_FAR_15_SHOOT));
        RSF.setLinearHeadingInterpolation(BLUE_FAR_EAT_1.getHeading(), BLUE_FAR_15_SHOOT.getHeading());

        SE = new Path(new BezierCurve(BLUE_FAR_15_SHOOT, BLUE_FAR_EAT_2_CP, BLUE_FAR_EAT_2));
        SE.setLinearHeadingInterpolation(BLUE_FAR_15_SHOOT.getHeading(), BLUE_FAR_EAT_2.getHeading());

        RSS = new Path(new BezierCurve(BLUE_FAR_15_EAT2, BLUE_FAR_EAT_2_CP, BLUE_FAR_15_SHOOT));
        RSS.setLinearHeadingInterpolation(BLUE_FAR_15_EAT2.getHeading(), BLUE_FAR_15_SHOOT.getHeading());
    }

    public void autonomousPathUpdate() {
        // 55초 경과 시 즉시 주차 모드 (Case 100) 진입
        if (opmodeTimer.getElapsedTimeSeconds() >= 55.0 && pathState < 100) {
            setPathState(100);
            stop_eatting();
            shoot_stop();
            SL.setPower(0); SR.setPower(0); SA.setPower(0);
            Path parkPath = new Path(new BezierLine(follower.getPose(), pos_const.RED_PARKING));
            parkPath.setLinearHeadingInterpolation(follower.getPose().getHeading(), pos_const.RED_PARKING.getHeading());
            follower.followPath(parkPath, true);
        }

        switch (pathState) {
            case 0: // 슈터 대기
                if (isShooterReady()) setPathState(1);
                break;

            case 1: // 프리로드 샷 후 FE(수집1) 시작
                shoot();
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shoot_stop();
                    follower.followPath(FE);
                    setPathState(2);
                }
                break;

            case 2: // FE 이동 완료 대기 (수집 중)
                eatting();
                if (!follower.isBusy()) {
                    follower.followPath(RSF);
                    setPathState(3);
                }
                break;

            case 3: // RSF 이동 완료 대기 (슈팅 위치 1로 이동)
                if (!follower.isBusy()) setPathState(4);
                break;

            case 4: // 슈팅 1 후 SE(수집2) 시작
                shoot();
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shoot_stop();
                    follower.followPath(SE);
                    setPathState(5);
                }
                break;

            case 5: // SE 이동 완료 대기 (수집 중)
                eatting();
                if (!follower.isBusy()) {
                    follower.followPath(RSS);
                    setPathState(6);
                }
                break;

            case 6: // RSS 이동 완료 대기 (슈팅 위치 2로 이동)
                if (!follower.isBusy()) setPathState(7);
                break;

            case 7: // 슈팅 2 후 다시 FE(수집1)로 이동 (무한 반복 루프)
                shoot();
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shoot_stop();
                    follower.followPath(FE);
                    setPathState(2); // FE 이동 대기 상태인 2번으로 점프
                }
                break;

            case 100: // 주차 상태
                if (!follower.isBusy()) setPathState(101);
                break;
        }
    }

    //================== 추가 함수들 ============================================
    //=========================================================================
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

    private void eatting() {
        eat.setPower(1);
    }

    private void stop_eatting() {
        eat.setPower(0);
    }

    private void shoot() {
        eat.setPower(1);
        servo_S.setPosition(servo_pos_const.servo_shoot_go);
    }

    private void shoot_stop() {
        //eat.setPower(0);
        servo_S.setPosition(servo_pos_const.servo_shoot_block);
    }

    private void eat_servo_up() {
        servo_eat.setPosition(servo_pos_const.servo_eat_up);
    }

    private void eat_servo_down() {
        servo_eat.setPosition(servo_pos_const.servo_eat_down);
    }
    // 슈터 속도 준비 여부 함수====
    private boolean isShooterReady() {
        double target = targetMotorVelocity * SHOOTER_POWER_RATIO;
        double current = SL.getVelocity();

        return Math.abs(target - current) <= 50;
    }
    // ==============
}
