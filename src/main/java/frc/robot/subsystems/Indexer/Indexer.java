package frc.robot.subsystems.Indexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    SparkFlex indexer = new SparkFlex(IndexerConstants.indexer_id, MotorType.kBrushless);
    SparkMax belt = new SparkMax(IndexerConstants.belt_id, MotorType.kBrushless);
    
    public enum INDEXER_STATES{
        SERIALIZE,
        IDLE
    }
    private INDEXER_STATES currentIndexerState = INDEXER_STATES.IDLE;

    public Indexer(){}
    @Override
    public void periodic(){
        switch (currentIndexerState) {
            case IDLE:
                indexer.set(0);    
                belt.set(0);

                break;
        
            case SERIALIZE:
                indexer.set(IndexerConstants.indexerPower);    
                belt.set(IndexerConstants.beltPower);
                break;
        }
        

    }
    public void setIndexerState(INDEXER_STATES indexerState){
        this.currentIndexerState = indexerState;
    }
}   
