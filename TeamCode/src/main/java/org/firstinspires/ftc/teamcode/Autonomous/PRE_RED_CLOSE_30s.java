package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_close_shot;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_close_start;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_eat_first;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_eat_first_CP;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_eat_second;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_eat_second_CP;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_open_eat;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_open_eat_wait;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.red_out;
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

@Autonomous(name = "AUTO_PRE_RED_CLOSE_30s", group = "2026Premiere", preselectTeleOp = "TELEOP_RED_Priemier")
public class PRE_RED_CLOSE_30s extends OpMode {

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



    private Path FS,E1,SS,E2,TS,OPwait,OP,OUT,RS;
    private PathChain First_Shoot,Second_Shoot;



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        segmentTime = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(red_close_start);

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

        autonomousPathUpdate();


        shooter.ShotResult result = shooter.calculateShot(current_robot_pos, RED_GOAL, SCORE_HEIGHT, current_robot_vel, SCORE_ANGLE);
        if (result != null&&pathState<19) {

            double StaticTargetPosTicks = tracking.fix_to_goal_RED(current_robot_pos);

            double offsetTicks = (result.turretOffset / (2 * Math.PI)) * SHOOTER_ANGLE_TPR * (105.0/25.0);

            finalTurretAngle = (int) round(StaticTargetPosTicks/* + offsetTicks*/);

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

            SL.setVelocity(targetMotorVelocity*0.64);
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
        FS = new Path(new BezierLine(red_close_start,red_close_shot));
        FS.setLinearHeadingInterpolation(red_close_start.getHeading(),red_close_shot.getHeading());

        E1 = new Path(new BezierCurve(red_close_shot,red_eat_first_CP,red_eat_first));
        E1.setLinearHeadingInterpolation(red_close_shot.getHeading(),red_eat_first.getHeading());

        SS = new Path(new BezierLine(red_eat_first,red_close_shot));
        SS.setLinearHeadingInterpolation(red_eat_first.getHeading(),red_close_shot.getHeading());

        First_Shoot = follower.pathBuilder()
                .addPath(E1)
                .addPath(SS)
                .build();

        E2 = new Path(new BezierCurve(red_close_shot,red_eat_second_CP,red_eat_second));
        E2.setLinearHeadingInterpolation(red_close_shot.getHeading(),red_eat_second.getHeading());

        TS = new Path(new BezierLine(red_eat_second,red_close_shot));
        TS.setLinearHeadingInterpolation(red_eat_second.getHeading(),red_close_shot.getHeading());

        Second_Shoot = follower.pathBuilder()
                .addPath(E2)
                .addPath(TS)
                .build();

        OPwait = new Path(new BezierLine(red_close_shot,red_open_eat_wait));
        OPwait.setLinearHeadingInterpolation(red_close_shot.getHeading(),red_open_eat_wait.getHeading());

        OP = new Path(new BezierLine(red_open_eat_wait,red_open_eat));
        OP.setLinearHeadingInterpolation(red_open_eat_wait.getHeading(),red_open_eat.getHeading());

        RS = new Path(new BezierLine(red_open_eat,red_close_shot));
        RS.setLinearHeadingInterpolation(red_open_eat.getHeading(),red_close_shot.getHeading());

        OUT = new Path(new BezierLine(red_close_shot,red_out));
        OUT.setLinearHeadingInterpolation(red_close_shot.getHeading(),red_out.getHeading());
    }

    public void autonomousPathUpdate() {  //경로 상태 관리하기
        switch (pathState) {
            case 0:
                follower.followPath(FS);
                setPathState(1);
                break;

            case 1:
                eatting();
                if (!follower.isBusy()){
                    shoot();
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    setPathState(3);
                }
                break;

            case 3:
                follower.followPath(First_Shoot);
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()){
                    shoot();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    setPathState(6);
                }
                break;

            case 6:
                follower.followPath(Second_Shoot);
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy()){
                    shoot();
                    setPathState(8);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    setPathState(9);
                }
                break;

            case 9:
                follower.followPath(OPwait);
                setPathState(10);
                break;

            case 10:
                if (!follower.isBusy()){
                    follower.followPath(OP);
                    setPathState(11);
                }
                break;

            case 11:
                if(pathTimer.getElapsedTimeSeconds()>=3){
                    follower.followPath(RS);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()){
                    shoot();
                    setPathState(13);
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    setPathState(14);
                }
                break;

            case 14:
                follower.followPath(OPwait);
                setPathState(15);
                break;

            case 15:
                if (!follower.isBusy()){
                    follower.followPath(OP);
                    setPathState(16);
                }
                break;

            case 16:
                if(pathTimer.getElapsedTimeSeconds()>=3){
                    follower.followPath(RS);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()){
                    shoot();
                    setPathState(18);
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds()>=1){
                    shoot_stop();
                    setPathState(19);
                }
                break;

            case 19:
                follower.followPath(OUT);
                setPathState(20);
                eat.setPower(0);
                SL.setPower(0);
                SR.setPower(0);
                break;
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
