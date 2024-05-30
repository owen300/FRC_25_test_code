package frc.robot;

import java.util.ArrayList;
import java.util.HashSet;

import edu.wpi.first.wpilibj2.command.CommandScheduler;//

public class State {//this is depriciated and will not be used most likely
    
    //todos are used for important notes lol
    //the benifit of this system is that all coridnation is done here 
    // making coridnation between subsystems more central and simple

    //TODO: IMPORTANT, USE WRAPPED COMMANDS FOR ALL MECHANISM COMMANDS
    //If the command should end in a state other than stable override the end meathod,
    // and set to the proper state

    //For example, when setting to shoot amp or speaker, 
    //you should make sure the current state is readytoamp/speaker before shooting

    private static HashSet<robotState> dontRunIfBlocked= new HashSet<robotState>();
    public static enum robotState{//first enum is current state, here are a couple sample ones to show how this years bot might use it
        Stable(blockingType.NONE),//the second enum decides how this state blocks other states, 
        Intake(blockingType.PARTIAL),//if wanted, you could make even more types of blocking
        Speaker_setpoint(blockingType.PARTIAL),
        Amp_setpoint(blockingType.PARTIAL),
        AutoAim(blockingType.NONE),
        Compact(blockingType.PARTIAL),
        Low_pos(blockingType.NONE),
        Shootspeaker(blockingType.NONE),
        Shootamp(blockingType.NONE),
        Outake(blockingType.NONE),
        readytoamp(blockingType.NONE),
        readytoshoot(blockingType.NONE);

        public final blockingType type;

        private robotState(blockingType type){
            this.type=type;
        }
    }
    public static enum blockingType{//types of blocking enum
        NONE,
        PARTIAL,
        FULL
    }
    public static robotState currentState=robotState.Stable;//current state

    public static void init() {//init blocking
        dontRunIfBlocked.add(robotState.Amp_setpoint);
        dontRunIfBlocked.add(robotState.Speaker_setpoint);
        dontRunIfBlocked.add(robotState.Compact);
        dontRunIfBlocked.add(robotState.Low_pos);
        dontRunIfBlocked.add(robotState.AutoAim);

    }

    public static void setState(robotState state){//normal set state
        if(!conflicts(state)){
       currentState=state;
       update();
        }
    }
    
    public static void setStateOverride(robotState state){//TODO: do not use unless needed, you dont want to break anything
       currentState=state;
       update();
    }

     public static void update(){//runs the commands
        switch(currentState){
         case Amp_setpoint://again, make sure your commands that need to end in something other than stable
         //override the end meathod and set the proper end state

         //each state can contain either a command or command group, if using a command group make sure only 
         //the last command sets the next state

            //put commands to move arm to amp, and all things involved with geting ready for amping here
            
           
            CommandScheduler.getInstance().schedule();//whatever commands needed in whatever order
            
            break;//repeat this for all states needed
        
        }
    }

    public static robotState getState(){
        return currentState;
    }

   public static boolean conflicts(robotState state){//check conficts
    if(currentState.type==blockingType.FULL){
        return true;
    } else if(currentState.type==blockingType.PARTIAL){
       return dontRunIfBlocked.contains(state);
    }
        return false;
   }
    
}
