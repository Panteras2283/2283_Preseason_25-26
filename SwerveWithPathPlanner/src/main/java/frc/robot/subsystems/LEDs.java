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

  public void Default(){

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

  public void Coral(){

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



  public void L2(){

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


  public void L3(){

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


  public void L4(){

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

  public void Algae(){

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


  public void Feeding(){

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
  
  public void ReefAlgae1(){

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

  public void ReefAlgae2(){

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

  public void FloorAlgae(){

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

  public void Net(){

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

  public void Processor(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=25";

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
      String targetUrl = "http://10.22.83.100/win&PL=27";

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
      String targetUrl = "http://10.22.83.100/win&PL=29";

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

  /*public void Pos1(){

    try{
      String targetUrl = "http://10.22.83.100/win&PL=31";

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
      String targetUrl = "http://10.22.81.100/win&PL=33";

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
      String targetUrl = "http://10.22.83.100/win&PL=35";

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
      String targetUrl = "http://10.22.83.100/win&PL=37";

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
      String targetUrl = "http://10.22.83.100/win&PL=39";

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
      String targetUrl = "http://10.22.83.100/win&PL=41";

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
  }*/
}
