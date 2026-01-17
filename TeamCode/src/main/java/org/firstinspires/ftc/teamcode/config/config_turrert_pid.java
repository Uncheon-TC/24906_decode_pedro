package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable

@TeleOp(name = "config_turret_pid", group = "config")
public class config_turrert_pid extends OpMode {

    private PIDFController controller;
@Sorter(sort = 0)
    public static double p = 0;
@Sorter(sort = 1)
    public static double i = 0;
@Sorter(sort = 2)
    public static double d = 0;
    public final double f = 0;
@Sorter(sort = 3)
    public static double target_deg = 0;
    public double target_tick;
    public final double ticks_per_rev = 103.8;
    private PIDFCoefficients pidfCoefficients;
    private double motor_power;

    DcMotor SA;


    @Override
    public void init() {
        pidfCoefficients = new PIDFCoefficients(p,i,d,f);
        controller = new PIDFController(pidfCoefficients);

        SA = hardwareMap.dcMotor.get("SA");

        SA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        target_tick = (target_deg/360.0) * (105.0/25.0) * ticks_per_rev; //패널에서 목표각도 입력 -> tick으로 변환
        controller.setTargetPosition(target_tick); //pidf controller 상수 변경

        pidfCoefficients = new PIDFCoefficients(p,i,d,f);
        controller.setCoefficients(pidfCoefficients);

        double currentPos = SA.getCurrentPosition();
        controller.updatePosition(currentPos);

        double current_deg = (currentPos) / (ticks_per_rev * (105.0/25.0)) * 360.0;


        motor_power = controller.run();

        SA.setPower(motor_power);

        telemetry.addData("target deg: ", target_deg);
        telemetry.addData("error deg: ", target_deg - current_deg);
        telemetry.addData("target tick: ", target_tick);
        telemetry.addData("error tick: ", target_tick - controller.getError());
        telemetry.update();
    }
}

