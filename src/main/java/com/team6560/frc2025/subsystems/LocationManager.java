package com.team6560.frc2025.subsystems;

import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team6560.frc2025.controls.ButtonBoard;

public class LocationManager extends SubsystemBase {
    private ButtonBoard board;

    private ReefIndex currentReefIndex;
    private ReefSide currentReefSide;
    private ReefLevel currentReefLevel;

    private boolean reefMode;


    public LocationManager(ButtonBoard board){
        this.board = board; 

        this.currentReefIndex = null;
        this.currentReefLevel = null;
        this.currentReefSide = null;
        this.reefMode = true;
    }

    @Override
    public void periodic() {
        update();
    }

    /** Runs periodically to check if an update to our target position has been made.*/
    public void update() {
        if(board.getShift()){
            reefMode = false;
        }
        else{
            reefMode = true;
        }

        // Levels first. Checks top down. 
        if(board.getL2()){
            currentReefLevel = ReefLevel.L2; 
        } if(board.getL3()){
            currentReefLevel = ReefLevel.L3; 
        } if(board.getL4()){
            currentReefLevel = ReefLevel.L4; 
        }

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
            currentReefSide = ReefSide.LEFT; // test: port 8
        } else if(board.getScoreF()){
            currentReefIndex = ReefIndex.TOP_LEFT;
            currentReefSide = ReefSide.RIGHT; // test: port 4
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
    }

    public void reset(){
        this.currentReefIndex = null;
        this.currentReefSide = null;
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

    public boolean hasTarget(){
        return currentReefIndex != null && currentReefLevel != null && currentReefSide != null;
    }

    public boolean hasLevel(){
        return currentReefLevel != null;
    }

    public boolean inReefMode(){
        return reefMode;
    }
}