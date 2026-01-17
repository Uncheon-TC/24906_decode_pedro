package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Field;

@Configurable


@TeleOp(name = "config_motor", group = "config")
public class motor_config extends OpMode {

    Servo servo_s, servo_hood;
    DcMotor eat, SL, SR, SA;

    public static double tar_servo_s, tar_servo_hood;
    public static int tar_eat, tar_SLR, tar_SA;


    @Override
    public void init() {

        servo_s = hardwareMap.servo.get("servo_s");
        servo_hood = hardwareMap.servo.get("servo_hood");

        servo_s.setPosition(0.5);
        servo_hood.setPosition(0.5);

        eat = hardwareMap.dcMotor.get("eat");
        SL = hardwareMap.dcMotor.get("SL");
        SR = hardwareMap.dcMotor.get("SR");
        SA = hardwareMap.dcMotor.get("SA");


        eat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {
        servo_s.setPosition(tar_servo_s);
        servo_hood.setPosition(tar_servo_hood);

        eat.setTargetPosition(tar_eat);
        SL.setTargetPosition(tar_SLR);
        SR.setTargetPosition(tar_SLR);
        SA.setTargetPosition(tar_SA);

        eat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("current_servo_s: ", servo_s.getPosition());
        telemetry.addData("target_servo_s: ", tar_servo_s);
        telemetry.addData("current_servo_hood: ", servo_hood.getPosition());
        telemetry.addData("target_servo_hood: ", tar_servo_hood);

        telemetry.addData("current_eat: ", eat.getCurrentPosition());
        telemetry.addData("target_eat: ", tar_eat);
        telemetry.addData("current_SL: ", SL.getCurrentPosition());
        telemetry.addData("target_SL: ", tar_SLR);
        telemetry.addData("current_SR: ", SR.getCurrentPosition());
        telemetry.addData("target_SR: ", tar_SLR);
        telemetry.addData("current_SA: ", SA.getCurrentPosition());
        telemetry.addData("target_SA: ", tar_SA);

        telemetry.update();
    }
}
