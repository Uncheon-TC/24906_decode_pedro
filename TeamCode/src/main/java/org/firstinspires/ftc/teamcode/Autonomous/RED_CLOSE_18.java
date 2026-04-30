package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_EAT1;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_EAT2;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_EAT2_CP;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_EAT4;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_EAT4_CP;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_END;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_GATE1;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_GATE2;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_GATE3;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_SHOOT1;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_START;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_CLOSE_ST_SHOOT;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_GOAL;
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

@Autonomous(name = "AUTO_RED_CLOSE_18_V1", group = "2025-2026 Test_auto", preselectTeleOp = "TELEOP_RED_LIMELIGHT")
public class RED_CLOSE_18 extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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



    private Path first_shoot;
    private Path eat1;
    private Path eat2;
    private Path shoot1;
    private Path shoot2;
    private Path eat3;
    private Path shoot3;
    private Path to_end;
    private Path gateopen;


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        segmentTime = new Timer();
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
        follower.update();
        Pose current_robot_pos = follower.getPose();  //save to Pose
        Vector current_robot_vel = follower.getVelocity();

        autonomousPathUpdate();


        shooter.ShotResult result = shooter.calculateShot(current_robot_pos, RED_GOAL, SCORE_HEIGHT, current_robot_vel, SCORE_ANGLE);
        if (result != null) {

            double StaticTargetPosTicks = tracking.fix_to_goal_RED(current_robot_pos);

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

            double targetMotorVelocity = velocityToTicks(result.launchSpeed);

            SL.setVelocity(targetMotorVelocity*0.65);
            shooter_power = SL.getPower();
            SR.setPower(shooter_power);
        }


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

        first_shoot = new Path(new BezierLine(RED_CLOSE_START, RED_CLOSE_ST_SHOOT));
        first_shoot.setLinearHeadingInterpolation(RED_CLOSE_START.getHeading(), RED_CLOSE_ST_SHOOT.getHeading());

        eat1 = new Path(new BezierCurve(RED_CLOSE_ST_SHOOT, RED_CLOSE_EAT2_CP, RED_CLOSE_EAT2));
        eat1.setLinearHeadingInterpolation(RED_CLOSE_ST_SHOOT.getHeading(), RED_CLOSE_EAT2.getHeading());

        shoot1 = new Path(new BezierLine(RED_CLOSE_EAT2, RED_CLOSE_SHOOT1));
        shoot1.setLinearHeadingInterpolation(RED_CLOSE_EAT2.getHeading(), RED_CLOSE_SHOOT1.getHeading());

        eat2 = new Path(new BezierLine(RED_CLOSE_SHOOT1, RED_CLOSE_GATE1));
        eat2.setLinearHeadingInterpolation(RED_CLOSE_SHOOT1.getHeading(), RED_CLOSE_GATE1.getHeading());

        gateopen = new Path(new BezierLine(RED_CLOSE_GATE1,RED_CLOSE_GATE2));
        gateopen.setLinearHeadingInterpolation(RED_CLOSE_GATE1.getHeading(), RED_CLOSE_GATE2.getHeading());

        shoot2 = new Path(new BezierLine(RED_CLOSE_GATE2, RED_CLOSE_SHOOT1));
        shoot2.setLinearHeadingInterpolation(RED_CLOSE_GATE2.getHeading(), RED_CLOSE_SHOOT1.getHeading());

        eat3 = new Path(new BezierLine(RED_CLOSE_SHOOT1, RED_CLOSE_EAT1));
        eat3.setLinearHeadingInterpolation(RED_CLOSE_SHOOT1.getHeading(), RED_CLOSE_EAT1.getHeading());

        shoot3 = new Path(new BezierLine(RED_CLOSE_EAT1, RED_CLOSE_SHOOT1));
        shoot3.setLinearHeadingInterpolation(RED_CLOSE_EAT1.getHeading(), RED_CLOSE_SHOOT1.getHeading());

        to_end = new Path(new BezierLine(RED_CLOSE_SHOOT1, RED_CLOSE_END));
        to_end.setConstantHeadingInterpolation(Math.toRadians(270));
    }

    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                follower.followPath(first_shoot);
                setPathState(1);
                break;
            case 1: // 프리로드 샷 이동
                eatting();
                if(!follower.isBusy()){
                    shoot();
                    setPathState(2);
                }
                break;
            case 2: // preload shot & 가운데 기물 먹으러 이동
                if(pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    follower.followPath(eat1);
                    setPathState(3);
                }
                break;
            case 3: // 가운데 기물 먹고 슈팅포인트로 이동
                eatting();
                if(!follower.isBusy()){
                    follower.followPath(shoot1);
                    setPathState(4);
                }
                break;
            case 4: // 가운데 기물 슈팅
                if(!follower.isBusy()){
                    shoot();
                    setPathState(5);
                }
                break;
            case 5: // 게이트로 접근
                if(pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    follower.followPath(eat2);
                    setPathState(6);
                }
                break;
            case 6:
                eatting();
                if(!follower.isBusy()){
                    setPathState(7);
                }
                break;
            case 7: // 게이트오픈
                follower.followPath(gateopen);
                setPathState(8);
                break;
            case 8: // 게이트 기물 수집 후 슈팅포지션으로
                if(pathTimer.getElapsedTimeSeconds()>=2){
                    follower.followPath(shoot2);
                    setPathState(9);
                }
                break;
            case 9: //슈팅(9개째)
                if(!follower.isBusy()){
                    shoot();
                    setPathState(10);
                }
                break;
            case 10: //게이트로 두번째 접근
                if(pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    follower.followPath(eat2);
                    setPathState(11);
                }
                break;
            case 11:
                eatting();
                if(!follower.isBusy()){
                    setPathState(12);
                }
                break;
            case 12: // 게이트 두번째 오픈
                follower.followPath(gateopen);
                setPathState(13);
                break;
            case 13: // 게이트 두번째 기물 수집후 슈팅포지션으로
                if(pathTimer.getElapsedTimeSeconds()>=2){
                    follower.followPath(shoot2);
                    setPathState(14);
                }
                break;
            case 14: // 슈팅(12개째)
                if(!follower.isBusy()){
                    shoot();
                    setPathState(15);
                }
                break;
            case 15: // 게이트로 세번째 접근
                if(pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    follower.followPath(eat2);
                    setPathState(16);
                }
                break;
            case 16:
                eatting();
                if(!follower.isBusy()){
                    setPathState(17);
                }
                break;
            case 17: // 게이트 세번째 오픈
                follower.followPath(gateopen);
                setPathState(18);
                break;
            case 18: // 게이트 세번째 기물 수집후 슈팅포지션으로
                if(pathTimer.getElapsedTimeSeconds()>=2){
                    follower.followPath(shoot2);
                    setPathState(19);
                }
                break;
            case 19: // 슈팅(15개째)
                if(!follower.isBusy()){
                    shoot();
                    setPathState(20);
                }
                break;
            case 20: // 가까운 기물 접근
                if(pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    follower.followPath(eat3);
                    setPathState(21);
                }
                break;

            case 21: // 가까운 기물 먹고 슈팅포인트로 이동
                eatting();
                if(!follower.isBusy()){
                    follower.followPath(shoot3);
                    setPathState(22);
                }
                break;
            case 22: // 가까운 기물 슈팅(18)
                if(!follower.isBusy()){
                    shoot();
                    setPathState(23);
                }
                break;
            case 23: // 엔드포지션으로 이동
                if(pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    follower.followPath(to_end);
                    setPathState(24);
                }
                break;




        }
    }

    // ======================== 추가함수 ====================================
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


}

