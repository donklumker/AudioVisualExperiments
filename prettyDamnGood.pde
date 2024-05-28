import processing.sound.*;
import controlP5.*;

// Declare variables for audio input and Fast Fourier Transform (FFT)
AudioIn mic;
FFT fft;

// Number of points (objects) in the simulation
int numPoints = 512;

// Arrays to store properties of each point
PVector[] points = new PVector[numPoints]; // Position
PVector[] velocities = new PVector[numPoints]; // Velocity
PVector[] accelerations = new PVector[numPoints]; // Acceleration
float[] sizes = new float[numPoints]; // Size
float[] spectrum = new float[numPoints]; // Audio spectrum data
int[] clusterIDs = new int[numPoints]; // Cluster IDs for each point
boolean[] isNewCluster = new boolean[numPoints]; // Flag to mark newly formed cluster cubes

// Flocking behavior parameters
float maxSpeed = 1.0; // Default maximum speed
float maxForce = 0.3; // Reducing this number creates greater coagulation
float desiredSeparation = 30; // Default separation distance
float neighborDistance = 100; // Default neighbor distance

ControlP5 cp5;

void setup() {
  size(1920, 1080, P3D); // Set up the canvas size and renderer
  
  mic = new AudioIn(this); // Initialize audio input
  mic.start(); // Start capturing audio
  fft = new FFT(this); // Initialize Fast Fourier Transform
  fft.input(mic); // Set audio input for FFT

  // Initialize ControlP5
  cp5 = new ControlP5(this);

  // Add a slider to control the desiredSeparation
  cp5.addSlider("desiredSeparation")
     .setPosition(10, 10)
     .setSize(200, 20)
     .setRange(1, 400) // Set the range of the slider
     .setValue(desiredSeparation); // Set the default value

  // Add a slider to control the neighborDistance
  cp5.addSlider("neighborDistance")
     .setPosition(10, 40)
     .setSize(200, 20)
     .setRange(1, 400) // Set the range of the slider
     .setValue(neighborDistance); // Set the default value

  // Add a slider to control the maxForce
  cp5.addSlider("maxForce")
     .setPosition(10, 70)
     .setSize(200, 20)
     .setRange(0.001, 1) // Set the range of the slider
     .setValue(maxForce); // Set the default value

  // Initialize points, velocities, sizes, and clusters
  for (int i = 0; i < numPoints; i++) {
    points[i] = new PVector(random(width), random(height), random(-500, 500));
    velocities[i] = PVector.random3D().mult(random(1, maxSpeed));
    accelerations[i] = new PVector();
    sizes[i] = 10;
    clusterIDs[i] = i; // Each point starts in its own cluster
  }
}

void draw() {
  background(0); // Clear the background
  noStroke(); // Disable stroke for shapes
  ambientLight(255, 130, 130); // Set ambient light
  directionalLight(51, 102, 126, -1, 0, 0); // Set directional light
  
  // Analyze audio spectrum
  fft.analyze(spectrum);
  
  // Update flocking behavior parameters based on slider values
  desiredSeparation = cp5.getController("desiredSeparation").getValue();
  neighborDistance = cp5.getController("neighborDistance").getValue();
  maxForce = cp5.getController("maxForce").getValue();
  
  // Move and draw cubes
  for (int i = 0; i < numPoints; i++) {
    // Apply flocking behaviors
    PVector alignment = align(velocities[i], i);
    PVector cohesion = cohere(points[i], i);
    PVector separation = separate(points[i], i);
    
    // Map spectrum energy to behavior parameters
    float speedFactor = map(spectrum[20], 0, 0.1, 1,50); // Adjust band index as needed
    maxSpeed = speedFactor *2 ; // Adjust maximum speed based on spectrum energy
    
    float forceFactor = map(spectrum[3], 0, .01, .1, .03); // Adjust band index as needed
    maxForce = forceFactor; // Adjust maxForce based on spectrum energy
    
    float separationFactor = map(spectrum[2], 0, 0.01, 100, 400); // Adjust band index as needed
    desiredSeparation = separationFactor; // Adjust desiredSeparation based on spectrum energy
    println (spectrum[15]);
    // Update acceleration
    PVector acceleration = new PVector();
    acceleration.add(alignment);
    acceleration.add(cohesion);
    acceleration.add(separation);
    
    // Update velocity
    velocities[i].add(acceleration).limit(maxSpeed);
    
    // Update position with boundary constraints
    points[i].add(velocities[i]);
    constrainBoundary(points[i], velocities[i]);
    
    // Draw cube at point
    drawCube(points[i], sizes[i]);
  }
}

// Function to calculate alignment behavior
PVector align(PVector velocity, int index) {
  PVector sum = new PVector(); // Initialize sum vector
  int count = 0; // Initialize count
  
  // Loop through all points
  for (int j = 0; j < numPoints; j++) {
    if (j != index) { // Exclude current point
      float d = PVector.dist(velocity, velocities[j]); // Calculate distance between velocities
      if (d > 0 && d < neighborDistance) { // If distance within range
        sum.add(velocities[j]); // Add velocity to sum
        count++; // Increment count
      }
    }
  }
  
  if (count > 0) {
    sum.div(count); // Divide sum by count to get average
    sum.normalize().mult(maxSpeed).sub(velocity).limit(maxForce); // Apply alignment behavior
  }
  
  return sum;
}

// Function to calculate cohesion behavior
PVector cohere(PVector position, int index) {
  PVector sum = new PVector(); // Initialize sum vector
  int count = 0; // Initialize count
  
  // Loop through all points
  for (int j = 0; j < numPoints; j++) {
    if (j != index) { // Exclude current point
      float d = PVector.dist(position, points[j]); // Calculate distance between positions
      if (d > 0 && d < neighborDistance) { // If distance within range
        sum.add(points[j]); // Add position to sum
        count++; // Increment count
      }
    }
  }
  
  if (count > 0) {
    sum.div(count); // Divide sum by count to get average
    return seek(sum, position, velocities[index]); // Apply cohesion behavior
  } else {
    return new PVector(); // Return zero vector if no nearby points
  }
}

// Function to calculate separation behavior
PVector separate(PVector position, int index) {
  PVector sum = new PVector(); // Initialize sum vector
  int count = 0; // Initialize count
  
  // Loop through all points
  for (int j = 0; j < numPoints; j++) {
    if (j != index) { // Exclude current point
      float d = PVector.dist(position, points[j]); // Calculate distance between positions
      if (d > 0 && d < desiredSeparation) { // If distance within range
        PVector diff = PVector.sub(position, points[j]); // Calculate difference vector
        diff.normalize().div(d); // Normalize and weight by distance
        sum.add(diff); // Add to sum
        count++; // Increment count
      }
    }
  }
  
  if (count > 0) {
    sum.div(count); // Divide sum by count to get average
  }
  
  return sum;
}

// Function to apply seeking behavior
PVector seek(PVector target, PVector position, PVector velocity) {
  PVector desired = PVector.sub(target, position); // Calculate desired vector
  desired.normalize().mult(maxSpeed); // Normalize and scale by maximum speed
  PVector steer = PVector.sub(desired, velocity); // Calculate steering force
  steer.limit(maxForce); // Limit steering force
  return steer; // Return steering force
}

// Function to constrain points within boundaries and handle bouncing
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
  if (p.z < -500 + halfSize) {
    p.z = -500 + halfSize;
    v.z *= -1; // Reverse velocity to bounce off the wall
  } else if (p.z > 500 - halfSize) {
    p.z = 500 - halfSize;
    v.z *= -1; // Reverse velocity to bounce off the wall
  }
}

// Function to draw a cube at a given position with a given size
void drawCube(PVector p, float size) {
  float halfSize = size / 2; // Half the size of the cube
  
  // Draw cube
  fill(255); // Set fill color
  translate(p.x, p.y, p.z); // Translate to position
  box(size); // Draw cube
  translate(-p.x, -p.y, -p.z); // Translate back
}
