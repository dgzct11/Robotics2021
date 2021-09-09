// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

import frc.robot.subsystems.NavXGyro;

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
    public static final int left_x_axis = 0;
    public static final int right_y_axis = 5;
    public static final int right_x_axis = 4;
    public static final int left_motor_port = 0;
    public static  double max_motor_percent = 0.3;
    public static final Port mxp_port = Port.kMXP;
    public static final double angle_error = 0.1;
    public static final double spin_rate = 0.2;
    public static double angle_correction_multiplier = 1.1;
    public static final double min_joystick_correction_threshold = 0.6;
    public static final int a_button_num = 1;
    public static final int b_button_num = 2;
    public static final int x_button_num = 3;
    public static final int y_button_num = 4;
    
    //placeholder ports
    public static final int left_pad_num = 270;
    public static final int right_pad_num = 90;
    public static final int up_pad_num = 0;
    public static final int down_pad_num = 180;

    public static final int lb_button_num = 5;
    public static final int rb_button_num = 6;
    public static final double distance_error = 0.01;
    public static int driveMode = 0; //0 = tank, 1 = tankstraight
    public static boolean angle_fixed = false;
	public static double kvVoltSecondsPerMeter;
    

    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kTrackwidthMeters = 3;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final DigitalSource kLeftEncoderPort = null;
	public static final DigitalSource kLeftEncoderReversed = null;
	public static final DigitalSource kRightEncoderPort = null;
	public static final DigitalSource kRightEncoderReversed = null;
    public static final int PID_TYPE = 0;
    public static final double position_units_per_meter =(10277+11467)*(1/0.479);
    public static final double distance_between_wheels = 0.698;
    
}
