// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Networking extends SubsystemBase {
  /** Creates a new Networking. */
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("datatable");
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  Dictionary networkEntries = new Hashtable();
  public Networking() {
    
  
  }
  public NetworkTableEntry getEntry(String key){
    return table.getEntry(key);
  }
  public void setEntry(String key, boolean value){
    table.getEntry(key).setBoolean(value);
  }
  public void setEntry(String key, double value){
    table.getEntry(key).setDouble(value);
  }
  public void setEntry(String key, double[] value){
    table.getEntry(key).setDoubleArray(value);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
