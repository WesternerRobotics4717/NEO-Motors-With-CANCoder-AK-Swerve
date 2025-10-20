package frc.robot.subsystems.elevator;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor1;
    private final ProfiledPIDController kElevatorPID;
    private ElevatorFeedforward kElevatorFF;



    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/Slot0/P", ElevatorConstants.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/Slot0/I", ElevatorConstants.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/Slot0/D", ElevatorConstants.kD);
    private static final LoggedTunableNumber tunableSetpoint= new LoggedTunableNumber("Elevator/TunableSetpoint", 0.0);
    private static final LoggedTunableNumber kMaxV = new LoggedTunableNumber("Elevator/Slot0/MaxV", ElevatorConstants.kMaxVelocity);
    private static final LoggedTunableNumber kMaxA = new LoggedTunableNumber("Elevator/Slot0/MaxA", ElevatorConstants.kMaxAcceleration);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/Slot0/S", ElevatorConstants.kS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/Slot0/V", ElevatorConstants.kV);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/Slot0/A", ElevatorConstants.kA);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/Slot0/G", ElevatorConstants.kG);

    public Elevator() {
        elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorID);

        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        TrapezoidProfile.Constraints constraint = new TrapezoidProfile.Constraints(ElevatorConstants.kV, ElevatorConstants.kA);
        kElevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, constraint);
        kElevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
        
        this.setDefaultCommand(enableFFCmd());
    }

    @Override
    public void periodic() {

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            updatePIDandFF(kP.get(), kI.get(), kD.get(), kMaxV.get(), kMaxA.get(), kS.get(), kV.get(), kA.get(), kG.get());
            }, kP, kI, kD, kMaxV, kMaxA, kS, kV, kA, kG);
        
        SmartDashboard.putNumber("Elevator/Position", getEncoderReading());
        stopIfLimit();
    }

    private void stopIfLimit() {
        if (isOutOfBounds(elevatorMotor1.getMotorOutputStatus().getValueAsDouble())) {
          elevatorMotor1.setVoltage(0);
        }
    }

    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && getEncoderReading() >= ElevatorConstants.kMaxPosition)
            || (pInput < 0 && getEncoderReading() <= ElevatorConstants.kMinPosition);
    }

    public FunctionalCommand setVoltsCmd(double pVoltage){
        return new FunctionalCommand(
            () -> {
            },
            () -> {
                elevatorMotor1.setVoltage(pVoltage);
            },
            (interrupted) -> elevatorMotor1.setVoltage(0),
            () -> false,
            this);
    }

    public FunctionalCommand setPIDCmd(double pSetpoint) {
        return new FunctionalCommand(
            () -> {
                kElevatorPID.reset(getEncoderReading());
                kElevatorPID.setGoal(pSetpoint);
                SmartDashboard.putNumber("Elevator/Setpoint", pSetpoint);  
            },
            () -> {
              double encoderReading = getEncoderReading();
              double calculatedPID = kElevatorFF.calculate(kElevatorPID.getSetpoint().velocity);
              double calculatedFF = kElevatorPID.calculate(encoderReading, pSetpoint);
    
              elevatorMotor1.setVoltage(calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Elevator/Full Output", calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Elevator/PID Output", calculatedPID);
              SmartDashboard.putNumber("Elevator/FF Output", calculatedFF);
            },
            (interrupted) -> elevatorMotor1.setVoltage(0),
            () -> false,
            this);
    }

    public FunctionalCommand enableFFCmd() {
        return new FunctionalCommand(
            () -> {
            },
            () -> {
              double calculatedOutput = kElevatorFF.calculate(0);
              elevatorMotor1.setVoltage(calculatedOutput);
            },
            (interrupted) -> elevatorMotor1.setVoltage(kElevatorFF.calculate(0)),
            () -> false,
            this);
    }

     public FunctionalCommand setTunablePIDCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> {
              double encoderReading = getEncoderReading();
              double setpoint = tunableSetpoint.getAsDouble();
              double calculatedPID = kElevatorFF.calculate(kElevatorPID.getSetpoint().velocity);
              double calculatedFF = kElevatorPID.calculate(encoderReading, setpoint);
    
              elevatorMotor1.setVoltage(calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Elevator/Full Output", calculatedPID + calculatedFF);
              SmartDashboard.putNumber("Elevator/PID Output", calculatedPID);
              SmartDashboard.putNumber("Elevator/FF Output", calculatedFF);
            },
            (interrupted) -> elevatorMotor1.setVoltage(0),
            () -> false,
            this);
    }

    private double getEncoderReading() {
        return elevatorMotor1.getPosition().getValueAsDouble();
    }

    
    public void updatePIDandFF(double kP, double kI, double kD, double kMaxV, double kMaxA, double kS, double kV, double kA, double kG) {
        kElevatorPID.setPID(kP, kI, kD);
        kElevatorPID.setConstraints(new TrapezoidProfile.Constraints(kMaxV, kMaxA));
        kElevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }



   
}