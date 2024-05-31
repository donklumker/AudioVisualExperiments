import processing.sound.*;
import controlP5.*;
import nervoussystem.obj.*;

boolean record = false;

// Declare variables for audio input and Fast Fourier Transform (FFT)
AudioIn mic;
FFT fft;

// Number of points (objects) in the simulation
int numPoints = 2048;

// Arrays to store properties of each point
PVector[] points = new PVector[numPoints]; // Position
PVector[] velocities = new PVector[numPoints]; // Velocity
PVector[] accelerations = new PVector[numPoints]; // Acceleration
float[] sizes = new float[numPoints]; // Size
float[] spectrum = new float[numPoints]; // Audio spectrum data

// Flocking behavior parameters
float maxSpeed = 10.0; // Default maximum speed
float maxForce = 0.3; // Reducing this number creates greater coagulation
float desiredSeparation = 30; // Default separation distance
float neighborDistance = 1; // Default neighbor distance

// Settings for live adjusting bands
int bandA;
int bandB;
int bandC;

ControlP5 cp5;

// Timer variables for OBJ export
int exportInterval = 2000; // 5 seconds
int lastExportTime = 0;
int exportCounter = 0;

void setup() {
  fullScreen(P3D); // Set up the canvas in full screen mode with P3D renderer

  mic = new AudioIn(this); // Initialize audio input
  mic.start(); // Start capturing audio
  fft = new FFT(this, numPoints); // Initialize Fast Fourier Transform
  fft.input(mic); // Set audio input for FFT

  // Initialize ControlP5
  cp5 = new ControlP5(this);

  // Add sliders to control various parameters
  cp5.addSlider("neighborDistance").setPosition(10, 40).setSize(200, 20).setRange(1, 400).setValue(neighborDistance);
  cp5.addSlider("maxForce").setPosition(10, 70).setSize(200, 20).setRange(0.001, 1).setValue(maxForce);
  cp5.addSlider("bandA").setPosition(10, 100).setSize(200, 20).setRange(0, 16).setValue(bandA);
  cp5.addSlider("bandB").setPosition(10, 130).setSize(200, 20).setRange(0, 19).setValue(bandB);
  cp5.addSlider("bandC").setPosition(10, 160).setSize(200, 20).setRange(0, numPoints/3).setValue(bandC);

  // Initialize points, velocities, sizes
  for (int i = 0; i < numPoints; i++) {
    points[i] = new PVector(random(width), random(height), random(-500, 500));
    velocities[i] = PVector.random3D().mult(random(1, maxSpeed));
    accelerations[i] = new PVector();
    sizes[i] = 3;
  }
}

void draw() {
  background(#4A6248); // Clear the background
  noStroke(); // Disable stroke for shapes
  ambientLight(255, 130, 130); // Set ambient light
  directionalLight(51, 102, 126, -1, 0, 1); // Set directional light
  directionalLight(51, 102, 226, 1, 0, 1); // Set directional light

  // Analyze audio spectrum
  fft.analyze(spectrum);

  // Update flocking behavior parameters based on slider values
  neighborDistance = cp5.getController("neighborDistance").getValue();
  maxForce = cp5.getController("maxForce").getValue();

  // Precompute the spectrum-based parameters once per frame
  float speedFactor = map(spectrum[bandA], 0, .001, 1, 5);
  float forceFactor = map(spectrum[bandB], 0, .005, 1, 0);
  float separationFactor = map(spectrum[bandC], 0, 0.001, 0, 1600);
  maxSpeed = speedFactor * 5;
  maxForce = forceFactor;
  neighborDistance = separationFactor;

  // Loop through each point and update its properties
  for (int i = 0; i < numPoints; i++) {
    PVector position = points[i];
    PVector velocity = velocities[i];
    PVector acceleration = accelerations[i];

    // Reset acceleration
    acceleration.set(0, 0, 0);

    // Apply flocking behaviors
    applyBehaviors(position, velocity, acceleration, i);

    // Update velocity and position
    velocity.add(acceleration).limit(maxSpeed);
    position.add(velocity);
    constrainBoundary(position, velocity);
  }

  // Batch draw cubes
  fill(255); // Set fill color
  for (int i = 0; i < numPoints; i++) {
    drawCube(points[i], sizes[i]);
  }

  // Display FPS
  fill(255);
  textAlign(RIGHT, BOTTOM);
  textSize(16);
  text("FPS: " + int(frameRate), width - 10, height - 10);

  // Check if it's time to export an OBJ model
  if (millis() - lastExportTime >= exportInterval) {
    exportOBJ();
    lastExportTime = millis();
  }
}

void applyBehaviors(PVector position, PVector velocity, PVector acceleration, int index) {
  PVector alignment = new PVector();
  PVector cohesion = new PVector();
  PVector separation = new PVector();
  int alignmentCount = 0;
  int cohesionCount = 0;
  int separationCount = 0;

  for (int j = 0; j < numPoints; j++) {
    if (j != index) {
      float d = PVector.dist(position, points[j]);

      if (d < neighborDistance) {
        alignment.add(velocities[j]);
        alignmentCount++;
        cohesion.add(points[j]);
        cohesionCount++;
      }

      if (d < desiredSeparation) {
        PVector diff = PVector.sub(position, points[j]);
        diff.normalize().div(d);
        separation.add(diff);
        separationCount++;
      }
    }
  }

  if (alignmentCount > 0) {
    alignment.div(alignmentCount).normalize().mult(maxSpeed);
    alignment.sub(velocity).limit(maxForce);
    acceleration.add(alignment);
  }

  if (cohesionCount > 0) {
    cohesion.div(cohesionCount);
    cohesion = seek(cohesion, position, velocity);
    acceleration.add(cohesion);
  }

  if (separationCount > 0) {
    separation.div(separationCount);
    separation.limit(maxForce);
    acceleration.add(separation);
  }
}

PVector seek(PVector target, PVector position, PVector velocity) {
  PVector desired = PVector.sub(target, position);
  desired.normalize().mult(maxSpeed);
  PVector steer = PVector.sub(desired, velocity);
  steer.limit(maxForce);
  return steer;
}

void constrainBoundary(PVector p, PVector v) {
  float halfSize = sizes[0] / 2; // Half the size of the cubes

  // X-axis boundary check
  if (p.x < halfSize) {
    p.x = halfSize;
    v.x *= -1; // Reverse velocity to bounce off the wall
  } else if (p.x > width - halfSize) {
    p.x = width - halfSize;
    v.x *= -1; // Reverse velocity to bounce off the wall
  }

  // Y-axis boundary check
  if (p.y < halfSize) {
    p.y = halfSize;
    v.y *= -1; // Reverse velocity to bounce off the wall
  } else if (p.y > height - halfSize) {
    p.y = height - halfSize;
    v.y *= -1; // Reverse velocity to bounce off the wall
  }

  // Z-axis boundary check
  if (p.z < -2000 + halfSize) {
    p.z = -2000 + halfSize;
    v.z *= -1; // Reverse velocity to bounce off the wall
  } else if (p.z > 500 - halfSize) {
    p.z = 500 - halfSize;
    v.z *= -1; // Reverse velocity to bounce off the wall
  }
}

void drawCube(PVector p, float size) {
  float halfSize = size / 2; // Half the size of the cube

  // Draw cube
  pushMatrix();
  translate(p.x, p.y, p.z); // Translate to position
  box(size); // Draw cube
  popMatrix();
}

void exportOBJ() {
  String filename = "export_" + nf(exportCounter++, 4) + ".obj";
  beginRecord("nervoussystem.obj.OBJExport", filename);
  fill(255); // Set fill color
  for (int i = 0; i < numPoints; i++) {
    drawCube(points[i], sizes[i]);
  }
  endRecord();
}

void keyPressed() {
  if (key == 's' || key == 'S') {
    record = true;
  }
}
