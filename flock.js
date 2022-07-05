let flock;
let flock_size;
// let canvasW = 640;
// let canvasH = 360;
let canvasW = 1000;
let canvasH = 500;
let box1_color, box2_color;
let conv_flag = false;
let feed_flag = false;
let feed_color;
let feedPosition, feedPositionX, feedPositionY;

const mix =  ["#9B5DE5","#F15BB5","#FEE440","#00BBF9","#00F5D4", "#FFBE0B","#FB5607","#FF006E","#8338EC","#3A86FF"];
const rainbow = ["#FF0000","#FF8700","#FFD300","#DEFF0A","#A1FF0A","#0AFF99","#0AEFFF","#147DF5","#580AFF","#BE0AFF"];

function setup() {
  createCanvas(canvasW, canvasH);
  flock_size = 0;
  // createP("Boid Count: " + flock_size);
  createP("- Click and drag mouse to generate new boids");
  createP("- UP arrow to converge, DOWN arrow to let go");
  createP("- RIGHT arrow to drop feed, LEFT arrow to stop");

  box1_color = random(rainbow);
  box2_color = random(rainbow);
  feed_color = random(rainbow);
  feedPositionX = random(200, 800);
  feedPositionY = random(0, 250);

  flock = new Flock();
  // Add an initial set of boids into the system
  for (let i = 0; i < 20; i++) {
    let b = new Boid(width / 2,height / 2);
    flock.addBoid(b);
  }
}

function draw() {
  background('#219ebc');
  fill(box1_color);
  rect(220, 290, 200, 220, 20, 20);
  fill(box2_color);
  rect(800, -20, 220, 100, 20, 20);
  flock.run();
  keyPressed();
}

// Add a new boid into the System
function mouseDragged() {
  flock.addBoid(new Boid(mouseX, mouseY));
}

function keyPressed() {
  if (keyCode === UP_ARROW) {
    conv_flag = true;
  }
  else if (keyCode === DOWN_ARROW) {
    conv_flag = false;
  }
  else if (keyCode === RIGHT_ARROW) {
    feed_flag = true;
    fill(feed_color);
    circle(feedPositionX, feedPositionY, 20);
    feedPosition = createVector(feedPositionX, feedPositionY);
  }
  else if (keyCode === LEFT_ARROW) {
    feed_flag = false; 
    feedPositionX = random(200, 800);
    feedPositionY = random(0, 250);
    feed_color = random(rainbow);
  }
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock object
// Does very little, simply manages the array of all the boids

function Flock() {
  // An array for all the boids
  this.boids = []; // Initialize the array
}

Flock.prototype.run = function() {
  for (let i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
  }
}

Flock.prototype.addBoid = function(b) {
  this.boids.push(b);
  flock_size++;
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x, y) {
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 4.0;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.05; // Maximum steering force
  this.color = random(mix);
}

Boid.prototype.run = function(boids) {
  this.flock(boids);
  this.update();
  // this.borders();
  this.render();
}

Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids) {
  let sep = this.separate(boids);   // Separation
  let ali = this.align(boids);      // Alignment
  let coh = this.cohesion(boids);   // Cohesion
  let avo = this.avoid(boids);      // Avoid walls + Box
  let con = this.converge(boids);   // Converge 
  let fee = this.feed(boids);       // Feed
  // Arbitrarily weight these forces
  sep.mult(10.0);
  ali.mult(2.0);
  coh.mult(1.0);
  avo.mult(3.0);
  con.mult(10.0);
  fee.mult(10.0);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
  this.applyForce(avo);
  this.applyForce(con);
  this.applyForce(fee);
}

// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function(target) {
  let desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  let theta = this.velocity.heading() + radians(90);
  // fill(127);
  // fill('#F679E5');
  fill(this.color);
  // stroke(200);
  noStroke();
  push();
  translate(this.position.x, this.position.y);
  rotate(theta);
  // beginShape();
  // vertex(0, -this.r * 2);
  // vertex(-this.r, this.r * 2);
  // vertex(this.r, this.r * 2);
  // endShape(CLOSE);
  beginShape();
  vertex(0, -this.r * 3.5);
  vertex(-this.r, -this.r * 2);
  vertex(-this.r/3, 0);
  vertex(-this.r, this.r * 2);
  vertex(this.r, this.r * 2);
  vertex(this.r/3, 0);
  vertex(this.r, -this.r * 2);
  endShape();
  pop();
}

// Wraparound
Boid.prototype.borders = function() {
  if (this.position.x < -this.r)  this.position.x = width + this.r;
  if (this.position.y < -this.r)  this.position.y = height + this.r;
  if (this.position.x > width + this.r) this.position.x = -this.r;
  if (this.position.y > height + this.r) this.position.y = -this.r;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  let desiredseparation = 25.0;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, boids[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0,0);
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    let steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  let neighbordist = 50;
  let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < boids.length; i++) {
    let d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}

Boid.prototype.converge = function(boids) {
  let conv = createVector(0, 0);
  let sum = createVector(0, 0);
  let count = 0;

  if (conv_flag) { // create 
    console.log("Feeding");

    for (let i = 0; i < boids.length; i++) {
      let d = p5.Vector.dist(this.position,conv);
      if ((d > 0)) {
        sum.add(boids[i].position); // Add location
        count++;
      }
    }
  }

  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}

Boid.prototype.feed = function(boids) {
  let eat = createVector(0, 0);

  if (feed_flag) {
    eat = this.seek(feedPosition);
  }
  
  return eat;
}

Boid.prototype.avoid = function(boids) {
  let steer = createVector(0, 0);

  // Avoid Walls
  if (this.position.x <= 0) {
    steer.add(createVector(1, 0));
  }
  if (this.position.x > canvasW) { // width of canvas
    steer.add(createVector(-1, 0));
  }
  if (this.position.y <= 0) {
    steer.add(createVector(0, 1));
  }
  if (this.position.y > canvasH) { // height of canvas
    steer.add(createVector(0, -1));
  }

  // Avoid Bottom Box
  // rect(220, 290, 200, 220, 20, 20);
  // LeftSide
  if (this.position.x == 210 && this.position.y >= 280) {
    steer.add(createVector(0, 1));
  }
  // Top+
  if (this.position.x >= 210 && this.position.x <= 430 && this.position.y >= 280) {
    steer.add(createVector(0, -1));
  }
  if (this.position.x == 430 && this.position.y >= 280) {
    steer.add(createVector(0, 1));
  }

  // Avoid top right Box  
  // rect(800, -20, 220, 100, 20, 20);
  if (this.position.x == 790 && this.position.y <= 80 ) {
    steer.add(createVector(0, 1));
  }
  if (this.position.x >= 790 && this.position.x <= canvasW && this.position.y <= 90) {
    steer.add(createVector(-1, 0));
  }


  return steer;
}
