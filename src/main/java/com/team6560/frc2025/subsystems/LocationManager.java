package com.team6560.frc2025.subsystems;

import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team6560.frc2025.controls.ButtonBoard;

public class LocationManager extends SubsystemBase {
    private ButtonBoard board;

    private ReefIndex currentReefIndex;
    private ReefSide currentReefSide;
    private ReefLevel currentReefLevel;

    private boolean goSwitch;

    public LocationManager(ButtonBoard board){
        this.board = board; 

        this.goSwitch = false;
        this.currentReefIndex = null;
        this.currentReefLevel = null;
        this.currentReefSide = null;
    }

    @Override
    public void periodic() {
        update();
    }

    /** Runs periodically to check if an update to our target position has been made.*/
    public void update() {
        // Levels first. Checks top down. 
        if(board.getL1()){
            currentReefLevel = ReefLevel.L1;
        } if(board.getL2()){
            currentReefLevel = ReefLevel.L2;
        } if(board.getL3()){
            currentReefLevel = ReefLevel.L3;
        } if(board.getL4()){
            currentReefLevel = ReefLevel.L4;
        }

        // Reef indices.
        // Indices are mapped from A to L. counterclockwise from left and far right, driver perspective
        if(board.getScoreA()){
            currentReefIndex = ReefIndex.FAR_RIGHT;
            currentReefSide = ReefSide.LEFT;
        } else if(board.getScoreB()){
            currentReefIndex = ReefIndex.FAR_RIGHT;
            currentReefSide = ReefSide.RIGHT;
        } else if(board.getScoreC()){
            currentReefIndex = ReefIndex.TOP_RIGHT;
            currentReefSide = ReefSide.LEFT;
        } else if(board.getScoreD()){
            currentReefIndex = ReefIndex.TOP_RIGHT;
            currentReefSide = ReefSide.RIGHT;
        } else if(board.getScoreE()){
            currentReefIndex = ReefIndex.TOP_LEFT;
            currentReefSide = ReefSide.LEFT;
        } else if(board.getScoreF()){
            currentReefIndex = ReefIndex.TOP_LEFT;
            currentReefSide = ReefSide.RIGHT;
        } else if(board.getScoreG()){
            currentReefIndex = ReefIndex.FAR_LEFT;
            currentReefSide = ReefSide.LEFT;
        } else if(board.getScoreH()){
            currentReefIndex = ReefIndex.FAR_LEFT;
            currentReefSide = ReefSide.RIGHT;
        } else if(board.getScoreI()){
            currentReefIndex = ReefIndex.BOTTOM_LEFT;
            currentReefSide = ReefSide.LEFT;
        } else if(board.getScoreJ()){
            currentReefIndex = ReefIndex.BOTTOM_LEFT;
            currentReefSide = ReefSide.RIGHT;
        } else if(board.getScoreK()){
            currentReefIndex = ReefIndex.BOTTOM_RIGHT;
            currentReefSide = ReefSide.LEFT;
        } else if(board.getScoreL()){
            currentReefIndex = ReefIndex.BOTTOM_RIGHT;
            currentReefSide = ReefSide.RIGHT;
        }

        // Go switch.
        if(board.getIntake()){
            goSwitch = true;
        } 
    }

    public void reset(){
        this.goSwitch = false;
        this.currentReefIndex = null;
        this.currentReefLevel = null;
    }

    public ReefIndex getCurrentReefIndex() {
        return currentReefIndex;
    }

    public ReefLevel getCurrentReefLevel() {
        return currentReefLevel;
    }

    public ReefSide getReefSide(){
        return currentReefSide;
    }

    public boolean isGoSwitchPressed() {
        return goSwitch;
    }

    public boolean hasTarget(){
        return currentReefIndex != null && currentReefLevel != null && currentReefSide != null;
    }
}