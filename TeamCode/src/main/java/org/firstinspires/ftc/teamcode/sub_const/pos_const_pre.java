package org.firstinspires.ftc.teamcode.sub_const;

import com.pedropathing.geometry.Pose;

public class pos_const_pre {
    public static Pose blue_close_start = new Pose(32, 132, Math.toRadians(90));
    public static Pose blue_close_shot = new Pose(60, 83, Math.toRadians(180));
    public static Pose blue_eat_first = new Pose(29, 83, Math.toRadians(180));
    public static Pose blue_eat_second = new Pose(29, 59, Math.toRadians(180));
    public static Pose blue_eat_second_CP = new Pose(45, 58);
    public static Pose blue_open_eat_wait = new Pose(20,50,Math.toRadians(180));
    public static Pose blue_open_eat = new Pose(13,59,Math.toRadians(139.5));
    public static Pose blue_out = new Pose(40,83,Math.toRadians(180));

    public static Pose red_close_start = blue_close_start.mirror();
    public static Pose red_close_shot = blue_close_shot.mirror();
    public static Pose red_eat_first = blue_eat_first.mirror();
    public static Pose red_eat_second = blue_eat_second.mirror();
    public static Pose red_eat_second_CP = blue_eat_second_CP.mirror();
    public static Pose red_open_eat_wait = blue_open_eat_wait.mirror();
    public static Pose red_open_eat = blue_open_eat.mirror();
    public static Pose red_out = blue_out.mirror();
}
