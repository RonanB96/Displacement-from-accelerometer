//
// Accel_Graph2_gyro3.pde
// Written by Ronan Byrne, adapted from other sources referanced where approperiate
// Last updated 15/05/2016
/*
   This program reads in values over the serial port from a MSP432 when data is available. 
 The MSP432 will be sending over accelerometer and gyroscope values which are the processed
 and then graphed.To process the data, the user must specify the start and end of the movement 
 from an acceleration graph of the data reduce integration errors. The data points inbetween 
 the specified points are integrated into displacements and are graphed. The user can choose 
 both SI and AP graphs, just one of these or graph them against eachother to show the trajectory of
 the sensor during the recorded movement. The user can chose to save the raw and processed data to
 the tests directory.
 */

import processing.serial.*; //import the Serial library
// Some code required the java swing library to be static, others didn't
// if one wasn't included, it would result in errors so both are imported
import static javax.swing.JOptionPane.*;
import javax.swing.*;

Serial myPort;  //the Serial port object

// Define variables
String val, portid, name,test_info,
  date, Dir1, filename;
String [] folder; 
boolean draw, firstContact, firstDraw, port, saved, fileOpen;
float[] Y, Z, Xo, Yo, Yfilt, Zfilt, Gyrofilt, Xrfilt, Yrfilt, yG, zG, yr, xr, Zcos, Ysin, 
  iy, iz, ixr, iyr, iiy, iiz, iixr, iiyr, tempY, tempZ, tempG, tempXr, tempYr, gyro, igyro, 
  Ymed, Zmed, Gyromed, Xrmed, Yrmed;
float ymax, ymin, zmax, zmin, test, y, z, xo, yo, gyo, g, Zcosavg, Ysinavg, 
  zAvg, yAvg, xrAvg, yrAvg, min, max, integralZ, Zoff, Yoff, m2, c2, 
  integralY, integralXr, integralYr, integralG, A, gy, start, finish;
int i, row, size, N; 
Table result;

float Ts =0.001;// sample interval
float B = 0.02;// low pass Filter factor
int window = 15;// window size for median filter
String COM_PORT="COM33";
int max_num_of_samples = 7000;
int n = 300;//number of samples to get average;
int id = 0; 
int graph =10;
boolean reset = false; 
boolean selection = false;


void setup() {
  size(1200, 700); //make our canvas 1200 x 700 pixels big
  background(150); // make background gray
  rectMode(CENTER);
  fill(255);
  rect(width/2, height/2, width-100, height-100);
  textSize(30);
  fill(0, 102, 153);
  textAlign(CENTER, CENTER);
  text("Waiting For Input", width/2, height/2);

  i = 0;
  ymin=ymax=zmin=zmax=0.0;
  fileOpen=draw=firstContact=firstDraw=port= false;

  if (reset== false) {//if the software has just been open intialise
    // create tables
    result = new Table();

    result.addColumn("Sample");
    result.addColumn("Reference X Acceleration(mm/s^2)");
    result.addColumn("Reference Y Acceleration(mm/s^2)");
    result.addColumn("Reference Normalised X Acceleration(mm/s^2)");
    result.addColumn("Reference Normalised Y Acceleration(mm/s^2)");
    result.addColumn("SI Acceleration (mm/s^2)");
    result.addColumn("AP Acceleration (mm/s^2)");
    result.addColumn("Normalised SI Acceleration (mm/s^2)");
    result.addColumn("Normalised AP Acceleration (mm/s^2)");
    result.addColumn("Angular Velocity (degrees/s)");
    result.addColumn("Normalised Angular Velocity(degrees/s)");
    result.addColumn(" ");
    result.addColumn("Reference X Velocity(mm/s)");
    result.addColumn("Reference Y Velocity(mm/s)");
    result.addColumn("SI Velocity(mm/s)");
    result.addColumn("AP Velocity(mm/s)");
    result.addColumn("Angle (degrees)");
    result.addColumn(" ");
    result.addColumn("Reference X Displacement(mm)");
    result.addColumn("Reference Y Displacement(mm)");
    result.addColumn("SI Displacement(mm)");
    result.addColumn("AP Displacement(mm)");
    result.addColumn(" ");
    result.addColumn("Max SI Displacement(mm)");
    result.addColumn("Min SI Displacement(mm)");
    result.addColumn("Min AP Displacement(mm)");
    result.addColumn("Max AP Displacement(mm)");
    result.addColumn("Duration(s)");
    result.addColumn("Date");

    //allocate memory for floats
    tempY =new float[window];
    tempZ = new float[window];
    tempG = new float[window];
    tempYr= new float[window];
    tempXr = new float[window];
    gyro = new float[max_num_of_samples];
    igyro = new float[max_num_of_samples];
    Ysin = new float[max_num_of_samples];
    Zcos = new float[max_num_of_samples];
    yG = new float[max_num_of_samples];
    zG = new float[max_num_of_samples];
    xr = new float[max_num_of_samples];
    yr = new float[max_num_of_samples];
    Xo = new float[max_num_of_samples];
    Yo = new float[max_num_of_samples];
    Y = new float[max_num_of_samples];
    Z = new float[max_num_of_samples];
    Yfilt = new float[max_num_of_samples];
    Zfilt = new float[max_num_of_samples];
    Gyrofilt = new float[max_num_of_samples];
    Xrfilt = new float[max_num_of_samples];
    Yrfilt = new float[max_num_of_samples];
    Ymed = new float[max_num_of_samples];
    Zmed = new float[max_num_of_samples];
    Gyromed = new float[max_num_of_samples];
    Xrmed = new float[max_num_of_samples];
    Yrmed = new float[max_num_of_samples];
    iz = new float[max_num_of_samples];
    iy = new float[max_num_of_samples];
    ixr = new float[max_num_of_samples];
    iyr = new float[max_num_of_samples];
    iiy = new float[max_num_of_samples];
    iiz = new float[max_num_of_samples];
    iiyr = new float[max_num_of_samples];
    iixr = new float[max_num_of_samples];

    printArray(Serial.list());
    // Look through serial list for COM8(MSP432 Com port)
    while (port == false) {
      for ( i=0; i<Serial.list().length; i++) {
        if (Serial.list()[i].equals(COM_PORT) == true) {
          portid = Serial.list()[i];
          port = true;
        }
      }
      if (portid ==null) {// Alert the user that the COM8 isn't connected
        showMessageDialog(null, "Com port not conntected!!!", 
          "Alert", ERROR_MESSAGE);   
        delay(1000);
      }
    }

    //initialize the serial port and set the baud rate to 115200
    myPort = new Serial(this, portid, 115200);
    myPort.bufferUntil('\n');
    port = true;
    reset = false;
  }

  //intilise test info object
  JTextField jinfo = new JTextField();
  Object [] secondMessage = {
    "Test Info: ", jinfo 
  };

  boolean input_done = false;
  boolean newTest = false;
  name=test_info="";

  while (input_done ==false) {//Ask the user to enter the Test name
    name = showInputDialog(null, "Name:", 
      "Info for test", QUESTION_MESSAGE);

    if (name.equals("") ==false) {
      folder = (listFileNames(sketchPath(""+name+"/")));
      if (folder == null) {// Check if test is new 
        //if test is new ask user to enter info about the test
        newTest = true;
        while (test_info.equals("") ==true) {
          if (newTest == true) {
            showConfirmDialog(null, secondMessage, 
              "Info for New Test", OK_OPTION);
            test_info = jinfo.getText();
            String [] data ={name, test_info};
            //save info to txt file in test folder
            saveStrings(sketchPath(""+name+"/Info_About_Test.txt"), data);
          }
        }
      }
      input_done = true;
    }
  }
}

void draw() {
  // if data is being recieved, display this to the user
  if (firstContact == true) {
    fill(255);
    rect(width/2, height/2, width-100, height-100);
    fill(0, 102, 153);
    textSize(30);
    textAlign(CENTER, CENTER);
    text("Receiving Data", width/2, height/2);
  }
  // if all of the data has been recieved perfrom initial calculations
  else if ( draw == true && graph!=0) {
    if (id > 10) {// Check if it wasn't a false send
      date = day()+"/"+month()+"/"+year();
      
      result.setString(0, "Date", date);
      Yfilt[0] = Y[0];
      Zfilt[0] = Z[0];
      Yrfilt[0] = Yo[0];
      Xrfilt[0] = Xo[0];
      Gyrofilt[0] = gyro[0];
      int I;
      for ( i=1; i<=id; i++) {
        // Low pass filter data
        //Y(n) = (1-B)*Y(n-1)+B(X(n))  Exponential Moving Average (EMA)
        Yfilt[i] = (1-B)*Yfilt[i-1]+B*Y[i];
        Zfilt[i] = (1-B)*Zfilt[i-1]+B*Z[i];
        Yrfilt[i] = (1-B)*Yrfilt[i-1]+B*Yo[i];
        Xrfilt[i] = (1-B)*Xrfilt[i-1]+B*Xo[i];
        Gyrofilt[i] = (1-B)*Gyrofilt[i-1]+B*gyro[i];
        if (Z[i] > max) max = Z[i];
        if (Z[i] < min) min = Z[i];
      }
      println("min = "+ min);
      println("max = " +max);
      // median filter data
      for ( i=0; i<id; i++) {
        for (I=0; I<window; I++) {
          if ( i+I >= Z.length) {
            tempY[I] = Yfilt[i];
            tempZ[I] = Zfilt[i];
            tempG[I] = Gyrofilt[i];
            tempXr[I] = Xrfilt[i];
            tempYr[I] = Yrfilt[i];
          } else {
            tempY[I] = Yfilt[i+I];
            tempZ[I] = Zfilt[i+I];
            tempG[I] = Gyrofilt[i+I];
            tempXr[I] = Xrfilt[i+I];
            tempYr[I] = Yrfilt[i+I];
          }
        }
        Ymed[i] = median(window, tempY);
        Zmed[i] = median(window, tempZ);
        Gyromed[i] = median(window, tempG);
        Xrmed[i] = median(window, tempXr);
        Yrmed[i] = median(window, tempYr);
        g += Gyromed[i];
        if (i<=n) {
          xo += Xrmed[i];
          yo += Yrmed[i];
          y += Ymed[i];
          z += Zmed[i];
        }
      }
      println("finished filtering");
      // Calculate the average of these
      xo = xo/(n*1.0);
      yo = yo/(n*1.0);
      y = y/(n*1.0);
      z = z/(n*1.0);
      g=g/(id*1.0);

      gyo = degrees(atan2(y, z));// Calculate Initial Angle
      i = 0;
      integralG =((gyro[0]-g)*Ts)+gyo;
      igyro[0] = integralG;
      Ysin[0] = sin(radians(igyro[0]))*9810;
      Zcos[0] = cos(radians(igyro[0]))*9810;
      Zcosavg=Zcos[0];
      Ysinavg = Ysin[0];
      for (i = 1; i< id; i++) {// calculate angle and gravity vectors on both axes
        integralG = integralG + (gyro[i]-g)*Ts;
        igyro[i] = integralG;
        Ysin[i] = sin(radians(igyro[i]))*9810;
        Zcos[i] = cos(radians(igyro[i]))*9810;
        if (i<=n) {
          Zcosavg+=Zcos[i];
          Ysinavg+=Ysin[i];
        }
      }
      Zcosavg=Zcosavg/(n*1.0);
      Ysinavg = Ysinavg/(n*1.0);
      // calculate the offset due to any rotation about the X axis of the accelerometer
      Zoff = z-Zcosavg;
      Yoff = y-Ysinavg;

      for (i=0; i<=id; i++) {// Isolate the acceleration due to the movement
        Ysin[i] = Ysin[i]+Yoff;
        yG[i] = Y[i]-Ysin[i];
        Zcos[i] = Zcos[i] +Zoff;
        zG[i] = Z[i]-Zcos[i];
      }

      for (i = 0; i<id; i++) { // Save initial results
        xr[i] = Xo[i]-xo;
        yr[i] = Yo[i] - yo;
        result.setInt(i, "Sample", i);
        result.setFloat(i, "SI Acceleration (mm/s^2)", Y[i]);
        result.setFloat(i, "AP Acceleration (mm/s^2)", Z[i]);
        result.setFloat(i, "Angular Velocity (degrees/s)", gyro[i]-g);
        result.setFloat(i, "Reference X Acceleration(mm/s^2)", Xo[i]);
        result.setFloat(i, "Reference Y Acceleration(mm/s^2)", Yo[i]);
        result.setFloat(i, "Reference Normalised X Acceleration(mm/s^2)", xr[i]);
        result.setFloat(i, "Reference Normalised Y Acceleration(mm/s^2)", yr[i]);
        result.setFloat(i, "Normalised SI Acceleration (mm/s^2)", yG[i]);
        result.setFloat(i, "Normalised AP Acceleration (mm/s^2)", zG[i]);
        result.setFloat(i, "Angle (degrees)", igyro[i]);
        result.setFloat(i, "Normalised Angular Velocity(degrees/s)", Gyromed[i]);
        result.setFloat(0, "Duration(s)", id*1.0/1000.0);
      }
      graph = 0; // set graph variable to the Z acceleration graph
      graph(); // Graph the acceleration Z acceleration 
      start = finish = 0;
    } else { // id wasn't greater than 10, tell user the data can't be processed
      showMessageDialog(null, "No data to process, re-record and send over the data again");
      rectMode(CENTER);
      fill(255);
      rect(width/2, height/2, width-100, height-100);
      textSize(30);
      fill(0, 102, 153);
      textAlign(CENTER, CENTER);
      text("Waiting For Input", width/2, height/2);
      draw = false;
    }
    // Wait for the user to select the start and end of the acceleration, the start and end are marked with coloured circles, 
    // objects cannot be drawn to the screen without the "draw" finishing a loop meaning using a while loop wouldn't
    // this is why the draw if statement is split into two parts
  } else if (draw==true && selection == false) { 
    if (start >0 && finish>0 && keyPressed == true && key == ENTER |key==RETURN) {
      selection=true;
      graph =1; // set graph variable to graph SI and AP graph
      println("enter");
      draw = false;
    }
  } else if (selection == true) {
    integrate();//integrate accelerations into displacements

    result.setFloat(0, "Min SI Displacement(mm)", ymin);
    result.setFloat(0, "Max SI Displacement(mm)", ymax);
    result.setFloat(0, "Min AP Displacement(mm)", zmin);
    result.setFloat(0, "Max AP Displacement(mm)", zmax);

    graph();//graph data SI and AP Graph
  } 
  firstContact = false;
  if (keyPressed == true) {
    // if "s" is pressed, save the table as a csv file
    if (key == 's') {
      if (saved == true) {// if data is already saved tell user this
        showMessageDialog(null, "This data has already been saved", 
          "Info", INFORMATION_MESSAGE);
      } else {
        try {
          time = hour()+"_" + minute()+" "+day()+"-"+month()+"-"+year();
          filename = name+"("+time+")";
          Dir1 = sketchPath(name+"/");
          File f = new File(Dir1+filename+".csv");
          i = 1;
          while (saved == false) {
            if (f.exists()) {// check if file/directory exist 
              filename = name+"("+time+")("+i+")"; // if file "i" exists check if file "i+1" exists, loop until the file doesn't exist
              println(filename);
              f = new File(Dir1+filename+".csv");
              i++;
            } else { 
              saveTable(result, Dir1+filename+".csv");
              showMessageDialog(null, "Saved", "Info", INFORMATION_MESSAGE); //tell the user that the file has been saved
              saved = true;
            }
          }
        }
        catch(Exception e) {// if there was problem saving, tell the user
          e.printStackTrace();
          showMessageDialog(null, "There was a problem saving", 
            "Alert", ERROR_MESSAGE);
        }
      }
      delay(3000);
    } else if (key == '1') {// If 1 was pressed, draw both AP and SI graphs
      graph = 1;
      graph();
    } else if (key == '2') {// If 2 was pressed, draw SI graph
      graph = 2;
      graph();
    } else if (key == '3') {// If 3 was pressed, draw AP graph
      graph = 3;
      graph();
    } else if (key == '4') {// If 4 was pressed, draw APvsSI graph
      graph = 4;
      graph();
    } else if (key == 'r') {// If r is pressed, enter new test
      reset=true;
      setup();
    }
    delay(3000);
  }
}

// This function has been adapted from https://learn.sparkfun.com/tutorials/connecting-arduino-to-processing
// by B_E_N
void serialEvent( Serial myPort) {
  //put the incoming data into a String - 
  //the '\n' is our end delimiter indicating the end of a complete packet
  val = myPort.readStringUntil('\n');

  //make sure our data isn't empty before continuing
  if (val != null) {
    //trim whitespace and formatting characters (like carriage return)
    val = trim(val);
    println(val);
    // If the value is longer than 20 characters, its our data
    if (val.length() > 15) {
      // Split String into parts that are seperated by a space ','
      String[] acc = split(val, ',');
      // convert strings to floats, divide by bits and covert to acceleration
      // by multiplying by range and convert from g to mm/s/s
      // Acc = (bits/bit_range)*sensor_range*acceleration_due_to_gravity(mm/s/s) 
      Xo[id] = ((float(acc[0]))/pow(2, 15))*9810*2;
      Yo[id] = ((float(acc[1]))/pow(2, 15))*9810*2;
      Y[id] = ((float(acc[2]))/pow(2, 15))*9810*2;
      Z[id] = ((float(acc[3]))/pow(2, 15))*9810*2;
      // convert gyro strings to float, divide by bits and convert into
      // angular velocity by multiplying by range degrees/s 
      // Angular = Acc = (bits/bit_range)*sensor_range
      gyro[id] = ((float(acc[4]))/pow(2, 15))*250;
      //find the max and min of the Z acceleration to used from graphing later
      if (Z[id] > max) max = Z[id];
      if (Z[id] < min) min = Z[id];
      id++;
    } else if (val.equals("Done")==true) {
      draw = true;// if done is recieved, process data
    }

    //look for our 'B' string to start the handshake
    //if it's there, clear the buffer, and send a request for data
    if (firstContact == false) {
      if (val.equals("B") ==true) {
        myPort.clear();
        firstContact = true;
        // clear tables
        result.clearRows();
        id =0;
        draw=saved=firstDraw = false;
        // reset max and mins
        ymin=ymax=zmin=zmax=0.0;
        min = 1000000;
        max=-1000000;
        graph = 10;
        // write "A" to recieve data from MSP
        myPort.write("A");
        println("contact");
      }
    } else { //if we've already established contact, keep getting and parsing data

      // when you've parsed the data you have, ask for more:
      myPort.write("A");
    }
  }
}

// Integrates acceleration to displacement using numerical integration
void integrate() {
  i = 0;
  zAvg =yAvg =integralY = integralZ=
    integralYr=integralXr=integralG= 0.0;
  id = int(finish-start); // number of samples to be integrated and graphed specified by the user
  yAvg=zAvg=0;

  for (i=0; i<=id; i++) { // find the average of the data to cancel any underlying constants
    yAvg+=yG[(i+int(start))];
    zAvg+=zG[(i+int(start))];
  }
  yAvg= yAvg/(id*1.0);
  zAvg = zAvg/(id*1.0);

  for (i=0; i<=id; i++) {
    yG[i+int(start)]=yG[i+int(start)]-yAvg;
    zG[i+int(start)]=zG[i+int(start)]-zAvg;
  }
  yAvg=zAvg=0;

  // perfom first integral by numerical integration
  // Y(n) = Y(n-1)+X(i)*Ts
  for (i=0; i<=id; i++) {
    integralY = integralY + yG[i+int(start)]*Ts;
    iy[i] = integralY;
    integralZ = integralZ + zG[i+int(start)]*Ts;
    iz[i] = integralZ;
    integralYr = integralYr + yr[i+int(start)]*Ts;
    iyr[i] = integralYr;
    integralXr = integralXr + xr[i+int(start)]*Ts;
    ixr[i] = integralXr;
    zAvg = zAvg +iz[i];
    yAvg = yAvg + iy[i];
    xrAvg = xrAvg +ixr[i];
    yrAvg = yrAvg + iyr[i];
  }
  // find avg of each integral
  zAvg = zAvg/(id*1.0);
  yAvg= yAvg/(id*1.0);
  xrAvg = xrAvg/(id*1.0);
  yrAvg = yrAvg/(id*1.0);
  i = 0;
  integralY = integralZ= integralYr = integralXr=0;
  zmin = ymin = min = 100000;
  zmax = ymax = max = -100000;

  // perform second integral and save data to tables
  for ( i=0; i<=id; i++) {
    iy[i] = iy[i] - yAvg;
    iz[i] = iz[i] - zAvg;
    iyr[i] = iyr[i] - yrAvg;
    ixr[i] = ixr[i] - xrAvg;
    result.setFloat(i, "SI Velocity(mm/s)", iy[i]);
    result.setFloat(i, "AP Velocity(mm/s)", iz[i]);
    result.setFloat(i, "Reference X Velocity(mm/s)", ixr[i]);
    result.setFloat(i, "Reference Y Velocity(mm/s)", iyr[i]);

    integralY = integralY + iy[i]*Ts;
    iiy[i] = integralY;
    integralZ = integralZ + iz[i]*Ts;
    iiz[i] = integralZ;
    integralYr = integralYr + iyr[i]*Ts;
    iiyr[i] = integralYr;
    integralXr = integralXr + ixr[i]*Ts;
    iixr[i] = integralXr;
    result.setFloat(i, "SI Displacement(mm)", iiy[i]);
    result.setFloat(i, "AP Displacement(mm)", iiz[i]);
    result.setFloat(i, "Reference Y Displacement(mm)", iiyr[i]);
    result.setFloat(i, "Reference X Displacement(mm)", iixr[i]);

    // Find max and min of both axes
    if (iiy[i] > ymax) ymax = iiy[i];
    if (iiy[i] < ymin) ymin = iiy[i];
    if (iiz[i] > zmax) zmax = iiz[i];
    if (iiz[i] < zmin) zmin = iiz[i];
  }
  // find the large max and smaller min to used for graphing
  if (ymin < zmin) min = ymin;
  else min = zmin;
  if (ymax > zmax) max = ymax;
  else max = zmax;
}



void graph()// Graph data
{
  stroke(0);
  rectMode(CENTER);
  fill(255);
  rect(width/2, height/2, width-100, height-100);
  if (firstDraw ==false) {// set the size of the graph to be 20 percent greater than the max and min values
    min = min + 0.2*min;
    max = max + 0.2*max;
    firstDraw = true;
  }
  float yout, xout, zout;
  float oldxout, oldyout, oldzout;
  float m, c;
  oldyout=oldxout=oldzout=0;


  if (graph != 4) {// if not plotting APvsSI
    //Y axis formula
    m = (height-75 -75)/(min-max);  //y2-y1/x2-x1  
    c = 75 - max*m;                 //c = y1-x1*m


    //X axis formula
    m2 = (width-80-120)/(id*1.0);   //y2-y1/x2-x1
    c2 =width-80 - id*m2;           //c = y1-x1*m

    // Y axis label
    textSize(12);
    fill(0);
    textAlign(CENTER, RIGHT);
    text("0.000", c2-30, c-3);
    // Postive Label
    float marker = (c-75)/2+75; // find half way point on positive axis
    float num = (marker-c)/m;   // find value for this position
    textAlign(CENTER, RIGHT);
    if (max != 0.0000) {
      if (max > 0.3) { // if the minium value is very small dont print mid point marker
        text(nf(num, 1, 4), c2-30, marker);
      }
      text(nf(max, 1, 4), c2-30, 75);// write max value to axis
    }

    // Negetive Y axis Label
    if (min != 0.0000) {
      if (min < -0.3) { // if the minium value is very small dont print mid point marker
        marker = (height-75-c)/2+c;
        num = (marker-c)/m;
        text(nf(num, 1, 4), 90, marker);
      }
      text(nf(min, 1, 4), 90, height-75);
    }

    // Positive Y axis Label
    if (max != 0.0000) {
      if (max > 0.3) { // if the minium value is very small dont print mid point marker
        marker = (height-75-c)/2+c;
        num = (marker-c)/m;
        text(nf(num, 1, 4), 90, marker);
      }
      text(nf(max, 1, 4), 90, 75);
    }

    // Y axis title
    pushMatrix();
    translate(c2-50, height/2);
    rotate(HALF_PI);
    translate(-c2+50, -height/2);
    textAlign(CENTER, CENTER);
    if (graph == 0)text("Acceleration(mm/s^2)", c2-80, height/2+10);
    else text("Displacement(mm)", c2-80, height/2+10);
    popMatrix();


    //X axis label
    text("Seconds(s)", width/2, c+25);
    float xSection = round(id/250); //Split x axis into 0.25s intervals
    for (i = 1; i<= xSection; i++) {
      marker = i*250*m2+c2;
      num = (i*250.0)/1000.0;
      text(nf((num), 1, 3), marker, c+15);
    }
    // if the last value on the axis isn't within 0.1s of the last marker label last point 
    if (id/1000.0-num>100)text(nf(id/1000.0, 1, 3), width-80, c+15);
    textAlign(BOTTOM, RIGHT);
  } else {// if APvsSI is being graphed

    //Y axis formula
    m = (height-75 -75)/((ymin-ymax)+(ymin-ymax)*0.2);  //y2-y1/x2-x1  
    c = 75 - (ymax+ymax*0.2)*m;     //c = y1-x1*m

    //X axis formula
    m2 = (width-80-120)/((zmax-zmin)+(zmax-zmin)*0.2);   //y2-y1/x2-x1
    c2 =width-80 - (zmax+zmax*0.2)*m2;           //c = y2-x2*m
    textSize(12);
    fill(0);
    textAlign(CENTER, CENTER);
    text("Anterior(mm)", width/2, c+20);

    // Y axis label
    textAlign(CENTER, RIGHT);
    text("0.000", c2-30, c-3);
    // Postive Label
    float marker = (c-75)/2+75; // find half way point on positive axis
    float num = (marker-c)/m;   // find value for this position
    textAlign(CENTER, RIGHT);
    text(nf(num, 1, 4), c2-30, marker);
    text(nf(ymax+ymax*0.2, 1, 4), c2-30, 75);// write max value to

    marker = (height-75-c)/2+c;
    num = (marker-c)/m;
    text(nf(num, 1, 4), c2-30, marker);
    text(nf(ymin+ymin*0.2, 1, 4), c2-30, height-75);

    // X axis label
    if (zmax> 1) {
      for (i = 1; i<=3; i++) {
        marker = (width-80-c2)/3+c2; // find half way point on positive axis
        num = (i*marker+c2)/m2;   // find value for this position
        text(nf(num, 1, 4), i*marker, c+15);
      }
    }
    if (zmin< -1) {
      for (i = 1; i<=3; i++) {
        marker = (c2-120)/3+120; // find half way point on positive axis
        num = (i*marker+120)/m2;   // find value for this position
        text(nf(num, 1, 4), i*marker, c+15);
      }
    }
    text(nf(zmax+zmax*0.2, 1, 4), width-80, c+15);
    text(nf(zmin+zmin*0.2, 1, 4), 120, c+15);

    // Y axis title
    pushMatrix();
    translate(c2, height/2);
    rotate(HALF_PI);
    translate(-c2, -height/2);
    textAlign(CENTER, CENTER);
    text("Superior(mm)", c2-15, height/2+50);
    popMatrix();
  }

  line(120, c, width-80, c);// X axis line
  line(c2, 75, c2, height-75);// Y axis line

  if (graph!= 4 && graph!=0) {// if not plotting APvsSI or Z acceleration
    // plot first point
    yout = iiy[0]*m+c; //y=mx+c
    zout = iiz[0]*m+c;
    xout = i*(m2)+c2;
    stroke(22, 22, 204);
    point(0, yout);
    stroke(242, 10, 25);
    point(0, zout);
    oldyout = yout;
    oldzout = zout;
    oldxout = xout;
    for (i = 1; i< id; i++) {// plot the rest of the data
      yout = iiy[i]*m+c;
      zout = iiz[i]*m+c;
      xout = i*(m2)+c2;
      if ( graph != 3) {// Plotting both graphs or SI on its own
        stroke(22, 22, 204);
        line(xout, yout, oldxout, oldyout);
        textSize(20);
        fill(22, 22, 204);
        text("Y", width-150, 75);
        textSize(12);
        text("Y Max = "+ nf(ymax, 1, 3), width-150, 90);
        text("Y Min = "+ nf(ymin, 1, 3), width-150, 105);
      }
      if (graph != 2) {// Plotting both graphs or AP on its own
        stroke(242, 10, 25);
        line(xout, zout, oldxout, oldzout);
        textSize(20);
        fill(242, 10, 25);
        text("Z", width-260, 75);
        textSize(12);
        text("Z Max = "+ nf(zmax, 1, 3), width-260, 90);
        text("Z Min = "+ nf(zmin, 1, 3), width-260, 105);
      }
      oldxout = xout;
      oldzout = zout;
      oldyout = yout;
    }
  } else if (graph==0) { //plotting Z acceleration graph
    yout = Z[0]*m+c;
    zout = Zcos[0]*m+c;
    xout = i*(m2)+c2;
    stroke(22, 22, 204);
    point(0, yout);
    stroke(242, 10, 25);
    point(0, zout);
    oldyout = yout;
    oldxout = xout;
    oldzout=zout;
    textSize(20);
    fill(22, 50, 204);
    textAlign(CENTER, CENTER);
    text("Left Click the Start of the Movement and Right Click the end of the Movement", width/2, 65);
    text("Press Enter to Confirm", width/2, 90);

    for (i = 1; i< id; i++) {
      yout = Z[i]*m+c;
      zout = Zcos[i]*m+c;
      xout = i*(m2)+c2;
      stroke(22, 22, 204);
      line(xout, yout, oldxout, oldyout);
      stroke(242, 10, 25);
      line(xout, zout, oldxout, oldzout);
      oldxout = xout;
      oldzout=zout;
      oldyout = yout;
    }
  } else {// Plotting APvsSI
    yout = iiy[i]*m+c;
    zout = iiz[i]*m2+c2;
    stroke(22, 22, 204);
    point(zout, yout);
    oldyout = yout;
    oldzout = zout;

    for (i=1; i<id; i++) {
      yout = iiy[i]*m+c;
      zout = iiz[i]*m2+c2;
      stroke(22, 22, 204);
      line(zout, yout, oldzout, oldyout);

      textSize(20);
      fill(22, 22, 204);
      text("Superior", width-150, 75);
      fill(242, 10, 25);
      text("Anterior", width-260, 75);

      textSize(12);
      text("AP Max = "+ nf(zmax, 1, 3), width-260, 90);
      text("AP Min = "+ nf(zmin, 1, 3), width-260, 105);
      fill(22, 22, 204);
      text("SI Max = "+ nf(ymax, 1, 3), width-150, 90);
      text("SI Min = "+ nf(ymin, 1, 3), width-150, 105);
      oldzout = zout;
      oldyout = yout;
    }
  }
}

// List files in directory
String [] listFileNames(String dir) {

  File file = new File(dir);
  if (file.isDirectory()) {
    String names[] = file.list();
    return names;
  } else {
    // If it’s not a directory
    return null;
  }
}

// On closing check if data has been saved
void dispose() {
  if (saved ==false && id>0) {
    // ask user if they want ot save unsaved data
    int option = showConfirmDialog(null, "You have not saved current data, would you like to save it now?", 
      "Exiting Without Saving", YES_NO_OPTION);
    if ( option == YES_OPTION) {
      try {
        time = hour()+"_" + minute()+" "+day()+"-"+month()+"-"+year();
        filename = name+"("+time+")";
        Dir1 = sketchPath(name+"/");
        File f = new File(Dir1+filename+".csv");
        i = 1;
        while (saved == false) {
          if (f.exists()) {
            filename = name+"("+time+")("+i+")";
            println(filename);
            f = new File(Dir1+filename+".csv");
            i++;
          } else { 
            saveTable(result, Dir1 + filename+"result.csv");
            showMessageDialog(null, "Saved", "Info", INFORMATION_MESSAGE);
            saved = true;
          }
        }
      }
      catch(Exception e) {
        e.printStackTrace();
        showMessageDialog(null, "There was a problem saving", 
          "Alert", ERROR_MESSAGE);
      }
    }
  }
}


// Used to select the start and end of the swallow
void mousePressed() {
  if (graph==0) {
    if (mouseButton== LEFT) {
      start = (mouseX-c2)/m2;// x=(y-c)/m
      println(start);
      fill(255, 0, 0);
      ellipse(mouseX, mouseY, 20, 20); // mark start with red circle
    } else if (mouseButton == RIGHT) {
      finish = (mouseX-c2)/m2;
      println(finish);
      fill(0, 255, 0);
      ellipse(mouseX, mouseY, 20, 20); // mark end with green circle
    }
  }
}


// This function has been adapted from https://en.wikiversity.org/wiki/C_Source_Code/Find_the_median_and_mean
// Median filter
float median(int N, float x[]) {
  float temp;
  int I, j;
  // the following two loops sort the array x in ascending order
  for (I=0; I<N-1; I++) {
    for (j=I+1; j<N; j++) {
      if (x[j] < x[I]) {
        // swap elements
        temp = x[I];
        x[I] = x[j];
        x[j] = temp;
      }
    }
  }

  if (N%2==0) {
    // if there is an even number of elements, return mean of the two elements in the middle
    return((x[N/2] + x[N/2 - 1]) / 2.0);
  } else {
    // else return the element in the middle
    return x[N/2];
  }
}