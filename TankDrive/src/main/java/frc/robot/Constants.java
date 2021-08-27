// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int xbox_port = 0;
    public static final int right_motor_port = 4;
    
    public static final int left_y_axis = 1;
    public static final int right_y_axis = 5;
    public static final int right_x_axis = 0;
    public static final int left_motor_port = 0;
    public static final double max_motor_percent = .1;
    public static final Port mxp_port = Port.kMXP;
    public static final double angle_error = 0.6;
    public static final double spin_rate = 0.2;
    public static final double angle_correction_multiplier = 1.01;
    public static final double min_joystick_correction_threshold = 0;
    public static final int a_button_num = 1;
    public static final int b_button_num = 2;
    public static final int x_button_num = 3;
    public static int driveMode = 0; //0 = tank, 1 = tankstraight
}
