package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const_pre.blue_close_start;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.FLYWHEEL_TPR;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.HOOD_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SCORE_HEIGHT;
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
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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

@Autonomous(name = "AUTO_PRE_BLUE_CLOSE_60s", group = "2026Premiere", preselectTeleOp = "TELEOP_BLUE_Priemier")
public class PRE_BLUE_CLOSE_60s extends OpMode {

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private DcMotor eat, SA;
    private DcMotorEx SL, SR;
    private Servo servo_S, servo_hood, servo_eat;
    private Turret_Tracking tracking = new Turret_Tracking();
    private PIDFController controller;
    private int finalTurretAngle;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(blue_close_start);

        controller = new PIDFController(new PIDFCoefficients(shooter_p, shooter_i, shooter_d, shooter_f));

        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");
        SR  = hardwareMap.get(DcMotorEx.class, "SR");
        SA  = hardwareMap.dcMotor.get("SA");

        eat.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        eat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo_S = hardwareMap.servo.get("servo_S");
        servo_S.setPosition(servo_pos_const.servo_shoot_block);

        servo_hood = hardwareMap.servo.get("servo_H");
        servo_hood.setPosition(servo_pos_const.servo_hood_min);

        servo_eat = hardwareMap.servo.get("servo_EAT");
        servo_eat.setPosition(servo_pos_const.servo_eat_down);

        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients
                = new com.qualcomm.robotcore.hardware.PIDFCoefficients(flywheel_p, flywheel_i, flywheel_d, flywheel_f);
        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

        buildPaths();
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
        Pose current_robot_pos = follower.getPose();
        Vector current_robot_vel = follower.getVelocity();

        autonomousPathUpdate();

        // Shooter Logic
        shooter.ShotResult result = shooter.calculateShot(current_robot_pos, BLUE_GOAL, SCORE_HEIGHT, current_robot_vel, SCORE_ANGLE);
        if (result != null) {
            finalTurretAngle = tracking.fix_to_goal_BLUE(current_robot_pos);
            servo_hood.setPosition(mapAngleToServo(Range.clip(result.hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE)));

            controller.setTargetPosition(finalTurretAngle);
            controller.updatePosition(SA.getCurrentPosition());
            SA.setPower(controller.run());

            double targetTicks = velocityToTicks(result.launchSpeed);
            SL.setVelocity(targetTicks * 0.64);
            SR.setPower(SL.getPower());
        }

        pos_const.savedAutoPose = follower.getPose();
        updateTelemetry();
    }

    public void buildPaths() {
        // 경로 생성 로직 추가 공간
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // 첫 번째 경로 시작 예시
                // follower.followPath(yourPath);
                // setPathState(1);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private double mapAngleToServo(double angleRad) {
        double slope = (HOOD_SERVO_MIN - HOOD_SERVO_MAX) / (HOOD_MIN_ANGLE - HOOD_MAX_ANGLE);
        return slope * (angleRad - HOOD_MIN_ANGLE) + HOOD_SERVO_MIN;
    }

    private double velocityToTicks(double velocityInchesPerSec) {
        return (velocityInchesPerSec / (2 * Math.PI * WHEEL_RADIUS)) * FLYWHEEL_TPR;
    }

    private void updateTelemetry() {
        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("pathState", pathState);
        panelsTelemetry.update(telemetry);
    }
}
