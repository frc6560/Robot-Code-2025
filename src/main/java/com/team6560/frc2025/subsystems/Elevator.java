package com.team6560.frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.RobotContainer;
import edu.wpi.first.wpilibj.Encoder;

public class Elevator extends SubsystemBase{

    public final TalonFX ElevLeft;
    public final TalonFX ElevRight;

    public Elevator() {

        this.ElevLeft = new TalonFX(ElevatorConstants.ElevLeftCanID);
        this.ElevRight = new TalonFX(ElevatorConstants.ElevRightCanID);

    }

    public void setElevPos(double targetRotations){
        //Look at encoder positions
        double truepos = ElevLeft.getPosition().getValueAsDouble();
        if (truepos < targetRotations) {
            ElevLeft.setControl(new DutyCycleOut(0.2));
            ElevRight.setControl(new DutyCycleOut(-0.2));
        } else {
            stopElev();
        }
    }

    public void stopElev() {
        ElevLeft.stopMotor();
        ElevRight.stopMotor();
    }


}