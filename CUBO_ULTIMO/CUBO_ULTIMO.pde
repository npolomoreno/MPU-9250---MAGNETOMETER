//Programa para recibir los ángulos pitch, roll y yaw del puerto serie de arduino y 
//con ellos mover un cubo en 3D. 


import processing.serial.*;
Serial port;
String data="", acc="", gyro="", fusion="";
int index=0, index2=0;
float Pitch, Roll, Yaw;


void setup()  {
  printArray(Serial.list());
  port = new Serial(this, Serial.list()[2], 115200);
  port.bufferUntil('.');
  size(800, 800, P3D);
  noStroke();
  fill(204);
}

void draw()  {
  Pitch = float(acc);
  Roll = float(gyro);
  Yaw = float(fusion);
  
  background(0);
  lights();
  translate(width/2, height/2, 0);
  rotateZ(radians(-Pitch));
  rotateX(radians(Roll));
  box(160); 
}


void serialEvent(Serial port)
{
  data= port.readStringUntil('.');
  data=data.substring(0,data.length()-1);
  index= data.indexOf("*");
  index2=data.indexOf("@");
  acc= data.substring(0,index);                    //PITCH
  gyro= data.substring(index+1,index2);            // ROLL 
  fusion = data.substring(index2+1,data.length());   //YAW
}
