package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.IndexerSub;
import frc.robot.subsystems.hopper.IntakeSub;

public class HopperStructure extends SubsystemBase {

  private IndexerSub indexer;
  private IntakeSub intake;

  public enum HopperState {
    IDLE,
    INDEXING
  }

  private HopperState state = HopperState.IDLE;

  public HopperStructure(IndexerSub indexer, IntakeSub intake) {
    this.indexer = indexer;
    this.intake = intake;
  }

  public void requestIDLE() {
    state = HopperState.IDLE;
  }

  public void requestINDEXING() {
    state = HopperState.INDEXING;
  }

  @Override
  public void periodic() {
    switch (state) {
      case IDLE -> handleIdle();
      case INDEXING -> handleIndexing();
    }
  }

  private void handleIdle() {
    indexer.stop();
    intake.stop();
  }

  private void handleIndexing() {
    indexer.autofeed();
    intake.autofeed();
  }

  public void stopIndexing() {
    if (state == HopperState.INDEXING) {
      state = HopperState.IDLE;
    }
  }
}
