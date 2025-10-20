package com.team6560.frc2025.controls;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBoard {
    public final Joystick buttonBoard_1;
    public final Joystick buttonBoard_2;

    public ButtonBoard(int USB_ID_1, int USB_ID_2) {
        this.buttonBoard_1 = new Joystick(USB_ID_1);
        this.buttonBoard_2 = new Joystick(USB_ID_2);
    }

    public boolean getButton(int buttonID, Joystick buttonBoard) {
        return buttonBoard.getRawButton(buttonID);
    }

    public boolean getAxis(int axisID, Joystick buttonBoard) {
        return buttonBoard.getRawButtonPressed(axisID);
    }

    // Reef buttons
    public boolean getScoreA() {
        return buttonBoard_1.getRawButtonPressed(1);
    }
    public boolean getScoreB() {
        return buttonBoard_1.getRawButtonPressed(2);
    }
    public boolean getScoreC() {
        return buttonBoard_1.getRawButtonPressed(3);
    }
    public boolean getScoreD() {
        return buttonBoard_1.getRawButtonPressed(4);
    }
    public boolean getScoreE() {
        return buttonBoard_1.getRawButtonPressed(5);
    }
    public boolean getScoreF() {
        return buttonBoard_1.getRawButtonPressed(6);
    }
    public boolean getScoreG() {
        return buttonBoard_1.getRawButtonPressed(7);
    }
    public boolean getScoreH() {
        return buttonBoard_1.getRawButtonPressed(8);
    }
    public boolean getScoreI() {
        return buttonBoard_1.getRawButtonPressed(9);
    }
    public boolean getScoreJ() {
        return buttonBoard_1.getRawButtonPressed(10);
    }
    public boolean getScoreK() {
        return buttonBoard_1.getRawButtonPressed(11);
    }
    public boolean getScoreL() {
        return buttonBoard_1.getRawButtonPressed(12);
    }

    public boolean getL1(){
        return buttonBoard_2.getRawButton(2);
    }
    public boolean getL2(){
        return buttonBoard_2.getRawButton(3);
    }
    public boolean getL3(){
        return buttonBoard_2.getRawButton(4);
    }
    public boolean getL4(){
        return buttonBoard_2.getRawButton(5);
    }

    public boolean getShift(){
        return buttonBoard_2.getRawButton(6);
    }

    public boolean getIntake(){
        return buttonBoard_2.getRawButton(1);
    }
}
