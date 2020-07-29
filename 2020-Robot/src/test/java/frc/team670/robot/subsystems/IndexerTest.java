package frc.team670.robot.subsystems;

import org.junit.Test;
import org.junit.Assert;

import frc.team670.robot.subsystems.Indexer;

public class IndexerTest {
    
    @Test
    public void shortestDistDegreeTest(){
        Assert.assertEquals(0, Indexer.shortestDistDegrees(360, 0), 0);
        Assert.assertEquals(-144, Indexer.shortestDistDegrees(72, 288), 0);
        Assert.assertEquals(144, Indexer.shortestDistDegrees(72, 216), 0);
    }


}