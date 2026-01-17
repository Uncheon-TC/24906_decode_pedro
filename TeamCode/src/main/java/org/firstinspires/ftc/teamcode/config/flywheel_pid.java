package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable

@TeleOp(name = "config_flywheel_pid", group = "config")
public class flywheel_pid extends OpMode {

    DcMotorEx SL, SR;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double tar_vel = 0;

    private double lastP, lastI, lastD, lastF;

    @Override
    public void init() {
        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,i,d,f);

        lastP = p; lastI = i; lastD = d; lastF = f;

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        if (p != lastP || i != lastI || d != lastD || f != lastF) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
            SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // 변경된 값 기억
            lastP = p; lastI = i; lastD = d; lastF = f;
        }

        SL.setVelocity(tar_vel);
        SR.setVelocity(tar_vel);

        double cur_vel_SL = SL.getVelocity();
        double cur_vel_SR = SR.getVelocity();
        double cur_vel_avg = (cur_vel_SL + cur_vel_SR) / 2;
        double cur_vel_diff = cur_vel_SL - cur_vel_SR;


        double err_vel_SL = tar_vel - cur_vel_SL;
        double err_vel_SR = tar_vel - cur_vel_SR;
        double err_vel_avg = (err_vel_SL + err_vel_SR) / 2;
        double err_vel_diff = err_vel_SL - err_vel_SR;


        telemetry.addData("target vel: ", tar_vel);

        telemetry.addData("current vel(SL): ", cur_vel_SL);
        telemetry.addData("current vel(SR): ", cur_vel_SR);
        telemetry.addData("current vel(avg): ", cur_vel_avg);
        telemetry.addData("current vel(diff, L-R): ", cur_vel_diff);

        telemetry.addData("current err(SL): ", err_vel_SL);
        telemetry.addData("current err(SR): ", err_vel_SR);
        telemetry.addData("current err(avg): ", err_vel_avg);
        telemetry.addData("current err(diff, L-R)", err_vel_diff);

        telemetry.update();
    }

    @Override
    public void stop() {
        SL.setVelocity(0);
        SR.setVelocity(0);
    }
}
