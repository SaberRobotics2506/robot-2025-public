// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private Command m_autoSelected;
  private  SendableChooser<Command> auto_chooser;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("auto");

  public StringPublisher currentAutoCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());


    m_robotContainer = new RobotContainer();
    auto_chooser = AutoBuilder.buildAutoChooser("Do Nothing");

    try{

      auto_chooser.addOption("Single Reef Pro", new PathPlannerAuto("Single Reef Pro"));
      auto_chooser.addOption("Double Reef Pro", new PathPlannerAuto("Double Reef Pro"));
      auto_chooser.addOption("Triple Reef Pro", new PathPlannerAuto("Triple Reef Pro"));
      auto_chooser.addOption("2Reef", new PathPlannerAuto("2Reef"));
      auto_chooser.addOption("Straight", new PathPlannerAuto("Straight"));
      auto_chooser.addOption("TestKanwar", new PathPlannerAuto("TestKanwar"));
    }catch(Exception e){
      DataLogManager.log("Missing Auto FIle");
      DataLogManager.log(e.toString());
    }
    SmartDashboard.putData("Auto Choices", auto_chooser);
    
    currentAutoCommand = table.getStringTopic("currentAutoCommand").publish();
   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    currentAutoCommand.set(auto_chooser.getSelected().getName());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    GlobalVariables.limelightTolerance = 1000000000;
    m_autoSelected = auto_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected.getName());
    m_autoSelected.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    GlobalVariables.limelightTolerance = .1;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
  
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
