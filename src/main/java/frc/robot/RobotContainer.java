// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import frc.robot.subsystems.*;

import swervelib.SwerveInputStream;

public class RobotContainer {

    private final Joystick driver = new Joystick(0); 
  //  private final Joystick driver2 = new Joystick(1);
 
  
    private final JoystickButton zeroGyro =
    new JoystickButton(driver, 3);
  
    private final JoystickButton xLock = 
    new JoystickButton(driver, 6);
  /* 
    private final JoystickButton climb_In = 
    new JoystickButton(driver,7);
  
    private final JoystickButton climb_Bas =
    new JoystickButton(driver, 8 );
  
    private final JoystickButton shooter_tukuraminaaa =
    new JoystickButton(driver2, 1);
  
    private final JoystickButton shooter_alaminaaa =
    new JoystickButton(driver2, 2);
  
    private final JoystickButton shooter_Aci_Yukari = 
    new JoystickButton(driver2, 4);
  
    private final JoystickButton shooter_Aci_Asagi = 
    new JoystickButton(driver2, 3);
  
    private final JoystickButton elevator_l4Button = 
    new JoystickButton(driver2, 7);
  
    private final JoystickButton elevator_l3Button = 
    new JoystickButton(driver2, 9);
  
    private final JoystickButton elevator_l2Button =
    new JoystickButton(driver2, 11);
  
    private final JoystickButton elevator_kapat = 
    new JoystickButton(driver2, 8);
  
    private final JoystickButton shooter_duzelt = 
    new JoystickButton(driver2 , 10);

 */
  
  
    public final Swerve s_Swerve = new Swerve();
    

  /* 
    public final climb_in climb_in = new climb_in(s_yukari);
    public final climb_bas climb_bas = new climb_bas(s_yukari); 
  
  
  
    public final shooter_yukari shooter_aci_yukari = new shooter_yukari(s_yukari);
    public final shooter_asagi shooter_aci_asagi = new shooter_asagi(s_yukari);
  */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(s_Swerve.getSwerveDrive(), () -> -driver.getY(), () -> -driver.getX())
    .withControllerRotationAxis(() -> -Math.pow(driver.getRawAxis(4),3))

    .deadband(Constants.Swerve.stickDeadband)
    .scaleTranslation(0.8) // YAVASLATMA!!!!
    .allianceRelativeControl(true);
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driver.getRawAxis(2), () -> driver.getRawAxis(3)).headingWhile(false);
  
    Command FOdriveAngularVelocity = s_Swerve.driveFieldOriented(driveAngularVelocity);
    Command FOdriveDirectAngle = s_Swerve.driveFieldOriented(driveDirectAngle);
    // A chooser for autonomous commands
  
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    // The container for the robot. Contains subsystems, OI devices, and commands. 
    
    public RobotContainer() {
      /* 
      NamedCommands.registerCommand("Shooter_al", shooter_alamina(s_yukari));
      NamedCommands.registerCommand("shooter_tukur", shooter_tukuramina(s_yukari));
      NamedCommands.registerCommand("elevator_l4", elevator_otonom(s_yukari, -892, -42, -0.5)); 
      NamedCommands.registerCommand("elevator_l3", elevator_otonom(s_yukari, -456, -42, -0.5));
      NamedCommands.registerCommand("elevator_l2", elevator_otonom(s_yukari, -135, -37,-0.5));
      NamedCommands.registerCommand("erene_duzelt", elevator_otonom(s_yukari, -92, -11.5, -0.4));
      NamedCommands.registerCommand("elevator_kapa", elevator_kapat(s_yukari, -5.0, 0.2));
      NamedCommands.registerCommand("Shooter_duzelt", elevator_otonom(s_yukari, -0, -20,-0.3));
      */

      DriverStation.silenceJoystickConnectionWarning(true);
      s_Swerve.setDefaultCommand(FOdriveAngularVelocity);
      configureButtonBindings();
      m_chooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(m_chooser);
  
    
    }
 /*
   private Command shooter_tukuramina(Peripheral s_yukari){
      return new shooter_tukuramina(s_yukari);
   }
   private Command shooter_alamina(Peripheral s_yukari){
    return new shooter_tukuramina(s_yukari);
 }
 
    private Command elevator_otonom(Peripheral s_yukari,double position, double shooterTarget, double speed){
      return new elevator_ac(s_yukari,position,shooterTarget,speed);
    }
    private Command elevator_kapat(Peripheral s_yukari, double position, double speed) {
      return new elevator_kapa(s_yukari, position, speed);
    }
     */
  
    private void configureButtonBindings() {
  
        /* 
      shooter_duzelt.whileTrue(new elevator_ac(s_yukari, -54, -12, -0.4));
      shooter_duzelt.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));
  

      elevator_l4Button.whileTrue(new elevator_ac(s_yukari,  -892.0, -42,  -0.5));
      elevator_l4Button.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));
  
      elevator_l3Button.whileTrue(new elevator_ac(s_yukari, -456, -42,-0.4));
      elevator_l3Button.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));
  
      elevator_l2Button.whileTrue(new elevator_ac(s_yukari, -135, -37,-0.4));
      elevator_l2Button.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));
  
      shooter_Aci_Asagi.whileTrue(new RunCommand(() -> s_yukari.ShooteraciAsagi(), s_yukari));
      shooter_Aci_Asagi.whileFalse(new RunCommand(() -> s_yukari.ShooteraciDurdur(), s_yukari));
  
      shooter_Aci_Yukari.whileTrue(new RunCommand(()-> s_yukari.ShooteraciYukari(), s_yukari));
      shooter_Aci_Yukari.whileFalse(new RunCommand(()-> s_yukari.ShooteraciDurdur(), s_yukari));
  
      elevator_kapat.whileTrue(new elevator_kapa(s_yukari, -0.0, 0.2));
      elevator_kapat.whileFalse(new RunCommand(() -> s_yukari.elevatorDurdur(), s_yukari));
  
      shooter_tukuraminaaa.whileTrue(new shooter_tukuramina(s_yukari));
      shooter_tukuraminaaa.whileFalse(new shooter_duramina(s_yukari));

      shooter_alaminaaa.whileTrue(new shooter_alamina(s_yukari));
      shooter_alaminaaa.whileFalse(new shooter_duramina(s_yukari));
  
      climb_Bas.whileTrue(new RunCommand(() -> s_yukari.climbBas(), s_yukari));
      climb_Bas.whileFalse(new RunCommand(() -> s_yukari.climbDur(), s_yukari));
      
      climb_In.whileTrue(new RunCommand(() -> s_yukari.climbIn(), s_yukari));
      climb_In.whileFalse(new RunCommand(() -> s_yukari.climbDur(), s_yukari));
  */
      zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));  
      xLock.whileTrue(Commands.runOnce((() -> s_Swerve.lock()), s_Swerve).repeatedly());

    }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    s_Swerve.setMotorBrake(brake);
  }

  public void resetOdometry(Pose2d pose) {
    s_Swerve.resetOdometry(pose);
  }
  
  public void zeroGyro() {
    s_Swerve.zeroGyro();
  }

}
