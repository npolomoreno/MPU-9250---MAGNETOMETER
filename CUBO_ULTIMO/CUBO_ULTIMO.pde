//Programa para recibir los ángulos pitch, roll y yaw del puerto serie de arduino y 
//con ellos mover un cubo en 3D. 


import processing.serial.*;
Serial port;
String data="", acc="", gyro="", fusion="";
int index=0, index2=0;
float Pitch, Roll, Yaw;
PFont font1,font2;


void setup()  {
  printArray(Serial.list());
  port = new Serial(this, Serial.list()[2], 115200);
  port.bufferUntil('.');
  size(900, 900, P3D);
  noStroke();
  fill(200);
 font1=loadFont("CopperplateGothic-Bold-50.vlw");
 font2=loadFont("Bauhaus93-80.vlw");
}

void draw()  {
  
  //fill(40,200,255);
  
  
  Pitch = float(acc);
  Roll = float(gyro);
  Yaw = float(fusion);
  
 
  background(0);
  textFont(font2,60);
  text("mpu-9250",300,100);
  textFont(font2,24);
  fill(46,168,252);
  text("ACC, GYRO, MAGNETOMETER",280,150);
  textFont(font1,20);
  text("Developed by: npolo",100,800);
  //lights();
  lightSpecular(255, 255, 255);
  directionalLight(204, 204, 204, 0, 0, -1);
  specular(204, 102, 0);
  translate(width/2, height/2, 0);
  rotateZ(radians(-Pitch));
  rotateX(radians(Roll));
  //rotateY(radians(Yaw));
  box(200); 
  
  
  
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
