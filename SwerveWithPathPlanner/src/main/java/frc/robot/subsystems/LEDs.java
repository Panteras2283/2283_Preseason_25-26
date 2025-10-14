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

  public void NoCoralBlue(){

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

  public void NoCoralRed(){

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
}
