package frc.robot;




public class FireBirdsUtils{
    

    public int radsToCTRESensorUnits (double angleInRads, int encoderCPR) {

        return (int)((angleInRads) / (2*Math.PI) * encoderCPR);
    }
  
    public double CTRESensorUnitsToRads (double angleInSensorUnits, int encoderCPR) {
       
      return (double) ((angleInSensorUnits/encoderCPR)*2*Math.PI);
  
    }


}
