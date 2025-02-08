package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;


public class Elevator extends SubsystemBase {
    public enum State{
        STOW,
        L3,
        L4,
        BALL
    };
    
    private final TalonFX m_leftElev;
    private final TalonFX m_rightElev;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public Elevator() {
        this.m_leftElev = new TalonFX(ElevatorConstants.ELEV_LEFT_ID, "Canivore");
        this.m_rightElev = new TalonFX(ElevatorConstants.ELEV_RIGHT_ID, "Canivore");

        this.topLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_UPPER_LIMIT_SWITCH_ID);
        this.bottomLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_LOWER_LIMIT_SWITCH_ID);

        Slot0Configs elevatorPID = new Slot0Configs();
        // all random
            elevatorPID.kS = 0;
            elevatorPID.kG = 0.8;

            elevatorPID.kP = 0.75;
            elevatorPID.kI = 0.0;
            elevatorPID.kD = 0;
        
        m_leftElev.getConfigurator().apply(elevatorPID);
        m_rightElev.getConfigurator().apply(elevatorPID);


        ntDispTab("Elevator")
            .add("Elevator Height", this::getElevatorHeight)
            .add("Elevator angular velocity", this::getElevatorVelocity)
            .add("Upper limit switch", this::topLimitSwitchDown)
            .add("Bottom limit switch", this::bottomLimitSwitchDown);
    }

    @Override
    public void periodic() {
    }

    public void setElevatorPosition(double targetrotelev) {
        final PositionVoltage m_request = new PositionVoltage(targetrotelev);

        m_leftElev.setControl(m_request);
        m_rightElev.setControl(m_request);
    }


    public void stopMotors() {
        m_rightElev.stopMotor();
        m_leftElev.stopMotor();
    }

    public boolean topLimitSwitchDown() {
        return topLimitSwitch.get();
    }

    public boolean bottomLimitSwitchDown() {
        return bottomLimitSwitch.get();
    }

    public double getElevatorHeight(){
        return m_leftElev.getPosition().getValueAsDouble() * ElevatorConstants.ELEV_GEAR_RATIO;
    }

    public double getElevatorVelocity(){
        return m_leftElev.getVelocity().getValueAsDouble();
    }

    public void resetEncoderPos(double setposition) {
        m_leftElev.setPosition(setposition);
        m_rightElev.setPosition(setposition);
    }

    // all code below this point is for testing purposes
    public void turnOnMotors(){
        m_leftElev.setControl(new VelocityVoltage(200));
        m_rightElev.setControl(new VelocityVoltage(200));
    }

    public void turnOnMotorsNoPID(){
        m_leftElev.set(0.1);
        m_rightElev.set(0.1);
    }
    public void revMotorsNoPID(){
        m_leftElev.set(-0.1);
        m_rightElev.set(-0.1);
    }

    public void testMotor(double output){
        m_leftElev.set(output);
        m_rightElev.set(output);
    }
}