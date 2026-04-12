package org.firstinspires.ftc.teamcode.auto_cal;

import static org.firstinspires.ftc.teamcode.sub_const.pos_const.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.pos_const.RED_GOAL;
import static org.firstinspires.ftc.teamcode.sub_const.shooter_const.SHOOTER_ANGLE_TPR;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.sub_const.shooter_const;

public class Turret_Tracking {
    public Turret_Tracking() {}
    private final double GEAR_RATIO = 105.0/25.0;

    public int fix_to_goal_RED(Pose robot_pos) {
        double dy = RED_GOAL.getY() - robot_pos.getY()/* - shooter_const.turret_offset_y(robot_pos.getHeading())*/;
        double dx = RED_GOAL.getX() - robot_pos.getX()/* - shooter_const.turret_offset_x(robot_pos.getHeading())*/;
        double target_Rad = Math.atan2(dy, dx) - robot_pos.getHeading(); //모두 rad값
        return RadToTicks(target_Rad); //목표 엔코더 tick값 반환
    }


    public int fix_to_goal_BLUE(Pose robot_pos) {
        double dy = BLUE_GOAL.getY() - robot_pos.getY()/* - shooter_const.turret_offset_y(robot_pos.getHeading())*/;
        double dx = BLUE_GOAL.getX() - robot_pos.getX()/* - shooter_const.turret_offset_x(robot_pos.getHeading())*/;
        double target_Rad = Math.atan2(dy, dx) - robot_pos.getHeading(); //모두 rad값 반환 ㅇㅇ
        return RadToTicks(target_Rad);
    }

    public double getTargetHeading(Pose robot_pos) {
        double dy = RED_GOAL.getY() - robot_pos.getY()/* - shooter_const.turret_offset_y(robot_pos.getHeading())*/;
        double dx = RED_GOAL.getX() - robot_pos.getX()/* - shooter_const.turret_offset_x(robot_pos.getHeading())*/;
        return Math.toDegrees(Math.atan2(dy, dx) - robot_pos.getHeading());
    }

    int RadToTicks(double Rad){
        // -90도(-π/2)에서 450도(5π/2)까지의 범위로 정규화
        double MIN_ANGLE_RAD = -Math.PI / 2;      // -90도
        double MAX_ANGLE_RAD = 5 * Math.PI / 2;   // 450도
        double FULL_ROTATION = 2 * Math.PI;

        // -π/2 ~ 5π/2 범위로 정규화
        while (Rad < MIN_ANGLE_RAD) {
            Rad += FULL_ROTATION;
        }
        while (Rad > MAX_ANGLE_RAD) {
            Rad -= FULL_ROTATION;
        }

        double ticks = (Rad / FULL_ROTATION) * SHOOTER_ANGLE_TPR * GEAR_RATIO;

        return (int) Math.round(ticks);
    }

}
