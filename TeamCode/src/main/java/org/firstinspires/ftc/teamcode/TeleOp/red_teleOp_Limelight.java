package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.*;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.*;

import static java.lang.Math.round;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// ★ Limelight 추가 import
import com.qualcomm.hardware.limelightvision.LimelightMegaTag2;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.auto_cal.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.auto_cal.Turret_Tracking;
import org.firstinspires.ftc.teamcode.sub_const.pos_const;
import org.firstinspires.ftc.teamcode.sub_const.servo_pos_const;
import org.firstinspires.ftc.teamcode.sub_const.shooter_const;

@Configurable
@TeleOp(name = "RED TeleOp with Limelight", group = "2025-2026 Test OP")
public class red_teleOp_Limelight extends LinearOpMode {

    private TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double vel_off   = 0.65;
    public static double turret_off = 0;
    public static double off_changed = 0;

    // ★ Limelight 수동 위치 하드리셋 설정
    // 리셋 키: gamepad2.back (두 번째 패드의 뒤로가기 버튼)
    // 리셋 후에는 자동으로 핀포인트 오도메트리로 복귀
    private static final String LIMELIGHT_NAME = "limelight"; // 하드웨어 맵 이름

    // 리셋 상태 피드백용
    private String  llResetStatus     = "대기중";
    private long    llResetTimeMs     = 0;
    private boolean llResetSuccess    = false;

    private DcMotor    FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;
    private DcMotor    eat, SA;
    private DcMotorEx  SL, SR;
    private Servo      servo_S, servo_hood;
    private IMU        imu;

    // ★ Limelight
    private Limelight3A limelight;
    private boolean     limelightAvailable = false;

    Turret_Tracking tracking = new Turret_Tracking();
    private Follower follower;
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));

    private PIDFController  controller;
    private double          target_tick;
    private PIDFCoefficients pidfCoefficients;
    private double          motor_power;
    private int             finalTurretAngle;
    private double          targetMotorVelocity;
    private double          shooter_power;
    private int             shooter_status = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        pidfCoefficients = new PIDFCoefficients(shooter_p, shooter_i, shooter_d, shooter_f);
        controller       = new PIDFController(pidfCoefficients);

        // ─── 드라이브 모터 초기화 ───────────────────────────────────
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
        SA  = hardwareMap.dcMotor.get("SA");

        eat.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        eat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eat.setPower(0);

        servo_S = hardwareMap.servo.get("servo_S");
        servo_S.setPosition(servo_pos_const.servo_shoot_block);

        servo_hood = hardwareMap.servo.get("servo_H");
        servo_hood.setPosition(servo_pos_const.servo_hood_min);

        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients =
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                        flywheel_p, flywheel_i, flywheel_d, flywheel_f);
        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

        // ─── ★ Limelight 초기화 ────────────────────────────────────
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

            // MegaTag2 파이프라인 사용 (AprilTag 3D 위치 추정)
            limelight.pipelineSwitch(0);

            // IMU 데이터를 Limelight에 전달 (MegaTag2에서 heading 보조)
            // → 아래 루프에서 매 프레임 업데이트
            limelight.start();
            limelightAvailable = true;
        } catch (Exception e) {
            // Limelight가 없는 환경에서도 코드가 동작하도록 예외 처리
            limelightAvailable = false;
            telemetry.addLine("[경고] Limelight를 찾을 수 없습니다. 위치 보정 비활성화.");
        }

        waitForStart();

        // ─── 메인 루프 ────────────────────────────────────────────
        while (opModeIsActive()) {

            follower.update(); // 핀포인트 오도메트리 업데이트 (매 루프)

            // ★ Limelight IMU heading 전달 (MegaTag2 정밀도 향상 — 항상 업데이트)
            if (limelightAvailable) {
                limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
            }

            // ★ 위치 하드리셋 키: gamepad2.back
            // → Limelight AprilTag 좌표로 즉시 덮어쓰고, 이후 핀포인트 오도메트리로 복귀
            if (gamepad2.backWasPressed()) {
                hardResetPoseWithLimelight();
            }

            if (gamepad1.backWasPressed()) follower.setPose(savedAutoPose);

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) {
                follower.setPose(new Pose(72, 72, follower.getHeading()));
            }

            double botHeading_pin = follower.getHeading();
            double rotX = x * Math.cos(-botHeading_pin) - y * Math.sin(-botHeading_pin);
            double rotY = x * Math.sin(-botHeading_pin) + y * Math.cos(-botHeading_pin);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            FrontLeftMotor.setPower( (rotY + rotX - rx) / denominator * slow);
            BackLeftMotor.setPower(  (rotY - rotX - rx) / denominator * slow);
            FrontRightMotor.setPower((rotY - rotX + rx) / denominator * slow);
            BackRightMotor.setPower( (rotY + rotX + rx) / denominator * slow);

            // ─── 서보 / eat ──────────────────────────────────────
            if (gamepad1.left_bumper) {
                servo_S.setPosition(servo_pos_const.servo_shoot_go);
                eat.setPower(1);
            } else {
                servo_S.setPosition(servo_pos_const.servo_shoot_block);
            }

            if (gamepad1.x) eat.setPower(1);
            if (gamepad1.y) eat.setPower(0);

            // ─── vel_off / turret_off 조정 ───────────────────────
            if (gamepad2.dpadDownWasPressed()) vel_off   -= 0.01;
            if (gamepad2.dpadUpWasPressed())   vel_off   += 0.01;
            if (gamepad2.dpadRightWasPressed()) turret_off += 31;
            if (gamepad2.dpadLeftWasPressed())  turret_off -= 31;

            // ─── 슈터 계산 ────────────────────────────────────────
            shooter.ShotResult result = shooter.calculateShot(
                    follower.getPose(), RED_GOAL, SCORE_HEIGHT,
                    follower.getVelocity(), SCORE_ANGLE);

            if (result != null) {
                double StaticTargetPosTicks = tracking.fix_to_goal_RED(follower.getPose());
                double offsetTicks = (result.turretOffset / (2 * Math.PI))
                        * SHOOTER_ANGLE_TPR * (105.0 / 25.0);

                finalTurretAngle = (int) round(StaticTargetPosTicks - offsetTicks - turret_off);

                double clampedAngle  = Range.clip(result.hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
                double hood_servo_pos = mapAngleToServo(clampedAngle);
                servo_hood.setPosition(hood_servo_pos);

                controller.setTargetPosition(finalTurretAngle);
                double currentPos = SA.getCurrentPosition();
                controller.updatePosition(currentPos);
                motor_power = controller.run();

                if (Math.abs(finalTurretAngle - currentPos) < shooter_const.shooter_deadband) {
                    motor_power = 0;
                }
                SA.setPower(motor_power);
            }

            if (result != null) {
                switch (shooter_status) {
                    case 0:
                        SL.setVelocity(0);
                        SR.setVelocity(0);
                        if (gamepad1.aWasPressed()) shooter_status = 1;
                        break;

                    case 1:
                        targetMotorVelocity = velocityToTicks(result.launchSpeed);
                        SL.setVelocity(targetMotorVelocity * vel_off);
                        shooter_power = SL.getPower();
                        SR.setPower(shooter_power);
                        if (gamepad1.bWasPressed()) shooter_status = 0;
                        break;
                }
            }

            // ─── 텔레메트리 ───────────────────────────────────────
            ptelemetry.addData("tarVelo",            targetMotorVelocity);
            ptelemetry.addData("vel_off",             vel_off);
            ptelemetry.addData("tarVelo*vel_off",     targetMotorVelocity * vel_off);
            ptelemetry.addData("curVelo",             SL.getVelocity());
            ptelemetry.addData("curVelo_nonoff",      SL.getVelocity() / vel_off);

            ptelemetry.addData("x",       follower.getPose().getX());
            ptelemetry.addData("y",       follower.getPose().getY());
            ptelemetry.addData("heading", Math.toDegrees(follower.getHeading()));

            ptelemetry.addData("turret_target_angle",  tracking.getTargetHeading(follower.getPose()));
            ptelemetry.addData("turret_current_angle",
                    SA.getCurrentPosition() * 360 / (537.7 * (105.0 / 25.0)));
            ptelemetry.addData("turret_off", turret_off);
            ptelemetry.addData("pos_off",    turret_off * 360 / (537.7 * (105.0 / 25.0)));

            // ★ Limelight 리셋 상태 텔레메트리
            ptelemetry.addData("LL 리셋 [gamepad2.back]", limelightAvailable ? "사용가능" : "없음");
            ptelemetry.addData("LL 리셋 상태", llResetStatus);
            if (llResetSuccess) {
                ptelemetry.addData("리셋 후 경과(ms)", System.currentTimeMillis() - llResetTimeMs);
            }

            ptelemetry.update(telemetry);
        }

        // ─── 종료 ────────────────────────────────────────────────
        if (limelightAvailable) limelight.stop();
    }


    // ════════════════════════════════════════════════════════════════
    // ★ Limelight AprilTag 위치 하드리셋 메서드
    // ════════════════════════════════════════════════════════════════
    /**
     * gamepad2.back 버튼 입력 시 호출됩니다.
     *
     * Limelight MegaTag2가 감지한 AprilTag 기반 절대 좌표로
     * follower의 현재 Pose를 즉시 덮어씁니다. (하드리셋)
     *
     * 리셋 이후에는 별도 처리 없이 follower.update()가 매 루프
     * 핀포인트 오도메트리 기반으로 위치를 계속 추적합니다.
     *
     * AprilTag를 감지하지 못하면 Pose를 변경하지 않고 실패 메시지만 출력합니다.
     */
    private void hardResetPoseWithLimelight() {
        if (!limelightAvailable) {
            llResetStatus  = "실패: Limelight 없음";
            llResetSuccess = false;
            return;
        }

        LimelightMegaTag2 mt2 = limelight.getLatestRobotOrientationUpdate();

        if (mt2 == null) {
            llResetStatus  = "실패: 데이터 없음";
            llResetSuccess = false;
            return;
        }

        if (mt2.fiducialResults == null || mt2.fiducialResults.length == 0) {
            llResetStatus  = "실패: AprilTag 미감지";
            llResetSuccess = false;
            return;
        }

        // Limelight 좌표(미터, WPILib 기준) → PedroPathing 좌표(inches)
        // ※ 좌표계가 다를 경우 아래 변환 수정 필요
        double resetX   = mt2.pose.position.x * 39.3701; // m → inches
        double resetY   = mt2.pose.position.y * 39.3701;
        double resetYaw = mt2.pose.orientation.getYaw();  // 라디안

        // ★ 핀포인트 오도메트리 기준 위치를 AprilTag 측정값으로 즉시 덮어쓰기
        follower.setPose(new Pose(resetX, resetY, resetYaw));
        // → 다음 루프부터 follower.update()가 이 위치를 기점으로 핀포인트 추적 재개

        llResetTimeMs  = System.currentTimeMillis();
        llResetSuccess = true;
        llResetStatus  = String.format("성공! (%.1f\", %.1f\") 태그 %d개",
                resetX, resetY, mt2.fiducialResults.length);
    }


    // ════════════════════════════════════════════════════════════════
    // 기존 헬퍼 메서드
    // ════════════════════════════════════════════════════════════════
    public boolean check_shooting_zone(Pose pose) {
        if (pose.getY() >= Math.abs(pose.getX() - 72) + 72) return true;
        if (pose.getY() <= -Math.abs(pose.getX() - 72) + 72) return true;
        return false;
    }

    private double mapAngleToServo(double angleRad) {
        double slope = (HOOD_SERVO_MIN - HOOD_SERVO_MAX) / (HOOD_MIN_ANGLE - HOOD_MAX_ANGLE);
        return slope * (angleRad - HOOD_MIN_ANGLE) + HOOD_SERVO_MIN;
    }

    private double velocityToTicks(double velocityInchesPerSec) {
        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS;
        double revsPerSec = velocityInchesPerSec / wheelCircumference;
        return revsPerSec * FLYWHEEL_TPR;
    }
}
