float baseAngleX = 0;
float baseAngleY = 0;
float baseAngleZ = 0;

float endAngleX = 0;
float endAngleY = 0;
float endAngleZ = 0;

float linkLength = 200;

import processing.serial.*;
Serial myPort;

void setup() {
  size(800, 600, P3D);
  noStroke();
  println(Serial.list()); // Lista de puertos disponibles en consola
  // Ajusta el índice [0] si necesitas otro puerto
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('\n'); // Espera hasta recibir una línea completa
}

void draw() {
  background(30);
  lights();
  translate(width / 2, height / 2, 0);

  pushMatrix();

  // === Apply base joint rotations ===
  rotateX(baseAngleY);
  rotateY(0);
  rotateZ(baseAngleX);

  // --- Draw base cube (joint A) ---
  pushMatrix();
  fill(255, 100, 100);
  box(50);
  popMatrix();

  // --- Draw link ---
  pushMatrix();
  translate(0, -linkLength / 2, 0);
  fill(100, 255, 100);
  box(20, linkLength, 20);
  popMatrix();

  // --- Move to joint B position ---
  translate(0, -linkLength, 0);

  // === Apply end joint rotations ===
  rotateX(0);
  rotateY(0);
  rotateZ(0);

  // --- Draw second cube (joint B) ---
  pushMatrix();
  fill(100, 100, 255);
  box(40);
  popMatrix();

  popMatrix(); // End of all transformations
  
  
 // === Dibujar panel de información ===
  hint(DISABLE_DEPTH_TEST);  // asegura que el texto no quede oculto
  camera();                  // reset cámara para overlay

  // Panel de fondo
  fill(0, 150); // negro semitransparente
  noStroke();
  rect(5, 5, 300, 80, 10); // panel redondeado

  // Texto
  fill(255);
  textSize(16);
  text("📊 Ángulos recibidos", 15, 25);

  fill(255, 200, 200);
  text("Base → X: " + nf(degrees(baseAngleX), 1, 1) + "°" +
       " | Y: " + nf(degrees(baseAngleY), 1, 1) + "°", 15, 50);

  fill(200, 220, 255);
  text("Extremo → X: " + nf(degrees(endAngleX), 1, 1) + "°" +
       " | Y: " + nf(degrees(endAngleY), 1, 1) + "°" , 15, 70);

  hint(ENABLE_DEPTH_TEST);
}

void serialEvent(Serial myPort) {
  String input = myPort.readStringUntil('\n');
  if (input != null) {
    input = trim(input);
    String[] parts = split(input, ',');

    if (parts.length == 4) {
      int sensor = int(parts[0]);
      float angleX = radians(float(parts[1]));
      float angleY = radians(float(parts[2]));
      float angleZ = radians(float(parts[3]));

      if (sensor == 0) {
        baseAngleX = angleX;
        baseAngleY = angleY;
        //println("Base angles (deg): X=" + degrees(baseAngleX) + " Y=" + degrees(baseAngleY));
      } else if (sensor == 1) {
        endAngleX = angleX;
        endAngleY = angleY;
        //println("End effector angles (deg): X=" + degrees(endAngleX) + " Y=" + degrees(endAngleY));
      }
    }
  }
}
