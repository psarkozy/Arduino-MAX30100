import processing.serial.*;

final int WIDTH = 1200;
final int HEIGHT = 600;
final int CHANNELS = 2;
final String serialPort = "/dev/tty.usbmodemFD13131";
final color[] colors = {color(0, 0, 0), color(255, 0, 0), color(0, 255, 0), color(0, 0, 255)};
final int ABSMAX = 800;

float[][] series = new float[CHANNELS][WIDTH];
float heartRate = 0;
int spO2 = 0;
boolean beatDetected = false;
int ptr = 0;

Serial myPort;

void settings()
{
  size(WIDTH, HEIGHT);
}

void setup ()
{
  String attemptPort = serialPort;
  
  for (int i=0 ; i < Serial.list().length ; ++i) {
    String port = Serial.list()[i];
    if (port.matches(".+tty\\.usbmodem.+")) {
      attemptPort = port;
      break;
    }
  }
  
  println("Opening port " + attemptPort);
  
  myPort = new Serial(this, attemptPort, 115200);
  
  stroke(0);
  fill(0);
  textSize(8);
}

void draw ()
{
  if (beatDetected) {
    background(255, 200, 200);
    beatDetected = false;
  } else {
    background(255);
  }
  
  stroke(30);
  
  line(0, height/2, width, height/2);

  float maxv=0, minv;
  for (int s=0 ; s < CHANNELS ; ++s) {
    float[] samples = series[s];
    maxv = max(maxv, abs(max(samples)), abs(min(samples)));
    if (ABSMAX != -1) {
      maxv = min(maxv, ABSMAX);
    }
  }

  // Avoids map() errors
  if (maxv == 0) {
    maxv = 1;
  }
  minv = -maxv;

  for (int s=0 ; s < CHANNELS ; ++s) {
    stroke(colors[s]);

    float[] samples = series[s];
    float seriesMax = max(samples);
    
    text("ch " + s + " cur:" + samples[ptr] + " max:" + seriesMax + " min:" + min(samples), 0, 8 + 10 * s);

    boolean maxDisplayed = false;
    for (int i = 0 ; i < WIDTH ; ++i) {
      if (i > 0) {
        float ipy = HEIGHT - map(samples[i-1], minv, maxv, 0, HEIGHT);
        float iy = HEIGHT - map(samples[i], minv, maxv, 0, HEIGHT);
        
        if (abs(samples[i] - seriesMax) < 0.001 && !maxDisplayed) {
          text("v=" + samples[i], i, iy);
          maxDisplayed = true;
        }
          
        line(i - 1, ipy, i, iy);
      }
    }
  }
  
  text("Rate: " + heartRate, 200, 8);
  text("SpO2: " + spO2 + "%", 200, 18);
}
  
void serialEvent (Serial myPort)
{
  String sLine = myPort.readStringUntil('\n');
  
  if (sLine == null) {
    return;
  }

  if (sLine.substring(0, 2).equals("R:")) {
    String[] sValues = split(sLine.substring(2), ',');
  
    for (int i=0 ; i < sValues.length ; ++i) {
      float sample = float(sValues[i]);
    
      if (Float.isNaN(sample)) {
        continue;
      }
    
      series[i][ptr] = sample;
    }
    
    ptr = (ptr + 1) % WIDTH;
  } else if (sLine.substring(0, 2).equals("H:")) {
    heartRate = float(sLine.substring(2));
  } else if (sLine.substring(0, 2).equals("B:")) {
    beatDetected = true;
  } else if (sLine.substring(0, 2).equals("C:")) {
    println(sLine);
  } else if (sLine.substring(0, 2).equals("O:")) {
    spO2 = int(float(sLine.substring(2)));
  }
}