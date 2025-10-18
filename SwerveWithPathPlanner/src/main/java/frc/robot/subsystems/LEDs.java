// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public LEDs() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void BlueDefault(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=1";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void BlueCoral(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=7";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void RedDefault(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=2";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }
  public void RedCoral(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=3";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void L1(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=24";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }



  public void L2(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=4";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }


  public void L3(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=5";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }


  public void L4(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=6";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void BlueAlgae(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=8";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void RedAlgae(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=9";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Feeding(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=10";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }
  
  public void ReefAlgae1(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=11";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void ReefAlgae2(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=12";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void FloorAlgae(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=13";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Net(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=14";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Processor(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=15";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Auto(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=16";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Climb(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=17";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Pos1(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=18";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Pos2(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=19";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Pos3(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=20";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Pos4(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=21";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Pos5(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=22";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }

  public void Pos6(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=23";

      URL url = new URL(targetUrl);

      HttpURLConnection connection = (HttpURLConnection) url.openConnection();

      connection.setRequestMethod("GET");


      int responseCode = connection.getResponseCode();


      BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
      String line;
      StringBuilder responseContent = new StringBuilder();
      while((line = reader.readLine()) != null){
        responseContent.append(line);
      }
      reader.close();

      connection.disconnect();

    }catch (Exception e){
      e.printStackTrace();
    }
  }
}
