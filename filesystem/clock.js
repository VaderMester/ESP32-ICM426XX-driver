//Display
var WIDTH = 192;
var HEIGHT = 32;
var FPS = 25;
var currf = 0;

var N_GRAINS = 512;
var grainobj = { x: 0, y: 0, vx: 0, vy: 0, posid: 0, colour: 0 };


var month = new Array(12);
month[0] = "Jan";
month[1] = "Feb";
month[2] = "Mar";
month[3] = "Apr";
month[4] = "May";
month[5] = "Jun";
month[6] = "Jul";
month[7] = "Aug";
month[8] = "Sep";
month[9] = "Oct";
month[10] = "Nov";
month[11] = "Dec";

var blink = false;

var grain = new Array[N_GRAINS];
var xOffset = -0.000;
var yOffset = -0.000;
//var tzoffset = 2;
var AccelGyro = new Array(6);
var accelVector = new vector();

var red = GFX.colorMake(255, 0, 0, 0);
var lightgreen = GFX.colorMake(100, 255, 50, 0);
var magenta = GFX.colorMake(255, 100, 255, 0);
var yellow = GFX.colorMake(255, 255, 0, 0);
var cyan = GFX.colorMake(0, 255, 255, 0);

var myRED      = GFX.colorMake(255, 0, 0, 0);
var myGREEN    = GFX.colorMake(0, 255, 0, 0);
var myBLUE     = GFX.colorMake(0, 0, 255, 0);
var myMAGENTA  = magenta;
var myYELLOW   = yellow;
var myCYAN     = cyan;
var myBLACK    = GFX.colorMake(0, 0, 0, 0);
var myWHITE    = GFX.colorMake(255, 255, 255, 0);

var myCOLORS = {myRED,myCYAN,myMAGENTA,myYELLOW,myBLUE};

function vector(x, y, z) {
    this.xAxis = x;
    this.yAxis = y;
    this.zAxis = z;
}

function setup() {

    GFX.clearBuf();
    GFX.setFont(0);
    GFX.setFrameTime(1000/FPS);
    AccelGyro = cubeOS.getAccelGyro();
    accelVector.xAxis = AccelGyro[0];
    accelVector.yAxis = AccelGyro[1];
    accelVector.zAxis = AccelGyro[2];
    xOffset = (accelVector.XAxis * -1) * -1;
    yOffset = (accelVector.YAxis * -1) * -1;

    for (var i = 0; i < N_GRAINS; i++) { // For each sand grain...

        var imgIndex = 0;
        do {
            grainobj.x = Math.random(WIDTH * 256); // Assign random position within
            grainobj.y = Math.random(HEIGHT * 256); // the 'grain' coordinate space
            // Check if corresponding pixel position is already occupied...
            for (j = 0; (j < i) && (((grainobj.x / 256) != (grainobj.x / 256)) ||
                ((grainobj.y / 256) != (grainobj.y / 256))); j++);
            xIndex = (grainobj.y / 256) * WIDTH;
            yIndex = grainobj.x / 256;
        } while (GFX.getPixel(xIndex, yIndex) != 0); // Keep retrying until a clear spot is found
        GFX.drawPixel(xIndex, yIndex, 255); // Mark it
        grainobj.pos = (grainobj.y / 256) * WIDTH + (grainobj.x / 256);
        grainobj.vx = grainobj.vy = 0; // Initial velocity is zero

        grainobj.colour = myCOLORS[i % NUM_COLOURS];
        grain.push(grainobj);
    }
}

function getGrains() {
    accelVector.xAxis = AccelGyro[0];
    accelVector.yAxis = AccelGyro[1];
    accelVector.zAxis = AccelGyro[2];
    var accelX = (accelVector.XAxis * -1) + xOffset;
    var accelY = (accelVector.YAxis * -1) + yOffset;
    var accelZ = accelVector.ZAxis;

    var ax = -accelX / 256,       // Transform accelerometer axes
        ay = -accelY / 256,      // to grain coordinate space
        az = Math.abs(accelZ) / 2048;  // Random motion factor
    az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
    ax -= az;                         // Subtract motion factor from X, Y
    ay -= az;
    var az2 = az * 2 + 1;         // Range of random motion to add back in

    // ...and apply 2D accel vector to grain velocities...
    var v2, v;// Velocity squared, and Absolute velocity

    for (var i = 0; i < N_GRAINS; i++) {
        grain[i].vx += ax + Math.random(az2); // A little randomness makes
        grain[i].vy += ay + Math.random(az2); // tall stacks topple better!
        // Terminal velocity (in any direction) is 256 units -- equal to
        // 1 pixel -- which keeps moving grains from passing through each other
        // and other such mayhem.  Though it takes some extra math, velocity is
        // clipped as a 2D vector (not separately-limited X & Y) so that
        // diagonal movement isn't faster
        v2 = grain[i].vx * grain[i].vx + grain[i].vy * grain[i].vy;
        if (v2 > 65536) { // If v^2 > 65536, then v > 256
            v = Math.sqrt(v2); // Velocity vector magnitude
            grain[i].vx = 256.0 * grain[i].vx / v; // Maintain heading
            grain[i].vy = 256.0 * grain[i].vy / v; // Limit magnitude
        }
    }

    // ...then update position of each grain, one at a time, checking for
    // collisions and having them react.  This really seems like it shouldn't
    // work, as only one grain is considered at a time while the rest are
    // regarded as stationary.  Yet this naive algorithm, taking many not-
    // technically-quite-correct steps, and repeated quickly enough,
    // visually integrates into something that somewhat resembles physics.
    // (I'd initially tried implementing this as a bunch of concurrent and
    // "realistic" elastic collisions among circular grains, but the
    // calculations and volument of code quickly got out of hand for both
    // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)

    var oldidx, newidx, delta, newx, newy;

    for (i = 0; i < N_GRAINS; i++) {
        newx = grain[i].x + grain[i].vx; // New position in grain space
        newy = grain[i].y + grain[i].vy;
        if (newx > MAX_X) {              // If grain would go out of bounds
            newx = MAX_X;          // keep it inside, and
            grain[i].vx /= -2;             // give a slight bounce off the wall
        } else if (newx < 0) {
            newx = 0;
            grain[i].vx /= -2;
        }
        if (newy > MAX_Y) {
            newy = MAX_Y;
            grain[i].vy /= -2;
        } else if (newy < 0) {
            newy = 0;
            grain[i].vy /= -2;
        }

        oldidx = (grain[i].y / 256) * WIDTH + (grain[i].x / 256); // Prior pixel #
        newidx = (newy / 256) * WIDTH + (newx / 256); // New pixel #
        if ((oldidx != newidx) &&         // If grain is moving to a new pixel...
            img[newidx]) {                // but if that pixel is already occupied...
            delta = Math.abs(newidx - oldidx);   // What direction when blocked?
            if (delta == 1) {               // 1 pixel left or right)
                newx = grain[i].x;    // Cancel X motion
                grain[i].vx /= -2;            // and bounce X velocity (Y is OK)
                newidx = oldidx;        // No pixel change
            } else if (delta == WIDTH) {    // 1 pixel up or down
                newy = grain[i].y;    // Cancel Y motion
                grain[i].vy /= -2;            // and bounce Y velocity (X is OK)
                newidx = oldidx;        // No pixel change
            } else { // Diagonal intersection is more tricky...
                // Try skidding along just one axis of motion if possible (start w/
                // faster axis).  Because we've already established that diagonal
                // (both-axis) motion is occurring, moving on either axis alone WILL
                // change the pixel index, no need to check that again.
                if ((Math.abs(grain[i].vx) - Math.abs(grain[i].vy)) >= 0) { // X axis is faster
                    newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
                    if (!img[newidx]) {             // That pixel's free!  Take it!  But...
                        newy = grain[i].y;    // Cancel Y motion
                        grain[i].vy /= -2;            // and bounce Y velocity
                    } else { // X pixel is taken, so try Y...
                        newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
                        if (!img[newidx]) {           // Pixel is free, take it, but first...
                            newx = grain[i].x;  // Cancel X motion
                            grain[i].vx /= -2;          // and bounce X velocity
                        } else { // Both spots are occupied
                            newx = grain[i].x;  // Cancel X & Y motion
                            newy = grain[i].y;
                            grain[i].vx /= -2;          // Bounce X & Y velocity
                            grain[i].vy /= -2;
                            newidx = oldidx;      // Not moving
                        }
                    }
                } else { // Y axis is faster, start there
                    newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
                    if (!img[newidx]) { // Pixel's free!  Take it!  But...
                        newx = grain[i].x;    // Cancel X motion
                        grain[i].vy /= -2;            // and bounce X velocity
                    } else { // Y pixel is taken, so try X...
                        newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
                        if (!img[newidx]) { // Pixel is free, take it, but first...
                            newy = grain[i].y; // Cancel Y motion
                            grain[i].vy /= -2;         // and bounce Y velocity
                        } else { // Both spots are occupied
                            newx = grain[i].x; // Cancel X & Y motion
                            newy = grain[i].y;
                            grain[i].vx /= -2;         // Bounce X & Y velocity
                            grain[i].vy /= -2;
                            newidx = oldidx;     // Not moving
                        }
                    }
                }
            }
        }
        grain[i].x = newx; // Update grain position
        grain[i].y = newy;
        img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
        img[newidx] = 255;  // Set new spot
        grain[i].pos = newidx;
        //Serial.println(newidx);
    }
}

function addZero(i) {
    if (i < 10) {
        i = "0" + i;
    }
    return i;
}



function drawTime1(xoffset, years, mon, day, hours, min, timec, yearc, monc, dayc) {
    GFX.start();
    GFX.clearBuf();
    mon = mon.toString();
    day = day.toString();
    hours = hours.toString();
    min = min.toString();
    print(years + "." + mon + "." + day + " " + hours + ":" + min);
    GFX.printText(hours.substr(0, 1), xoffset + 0, 7, timec);
    GFX.printText(hours.substr(1, 1), xoffset + 7, 7, timec);
    GFX.printText(min.substr(0, 1), xoffset + 17, 7, timec);
    GFX.printText(min.substr(1, 1), xoffset + 24, 7, timec);
    if (blink) {
        GFX.printText(":", xoffset + 12, 7, timec);
    }
    GFX.printText(years, xoffset + 0, 15, yearc);
    GFX.printText(mon, xoffset + 4, 23, monc);
    GFX.printText(day, xoffset + 8, 30, dayc);
    GFX.end();
}

//From here the setup and main loop starts

setup();

while (true) {
    GFX.start();
    print("Updating clock");
    var time = new Date();
    //utc = time.getTime() + (time.getTimezoneOffset() * 60000);
    //print("TimeZone Offset:" + utc);
    //time = new Date(utc + (3600000 * tzoffset));
    var hours = addZero(time.getHours());
    var min = addZero(time.getMinutes());
    var mon = month[time.getMonth()];
    var years = time.getFullYear();
    mon = addZero(mon);
    var day = addZero(time.getDate());

    if(currf > FPS) {
        blink = !blink;
        currf = 0;
    }
    currf++;
    drawTime1(years, mon, day, hours, min, lightgreen, myWHITE, myWHITE, lightgreen);
    AccelGyro = cubeOS.getAccelGyro();
    print("ax: " + gyro[0] + "; ay: " + gyro[1] + "; az: " + gyro[2] + "; gx: " + gyro[3] + "; gy: " + gyro[4] + "; gz: " + gyro[5]);
    getGrains();
    for (var i = 0; i < N_GRAINS; i++) {
        var yPos = grain[i].pos / WIDTH;
        var xPos = grain[i].pos % WIDTH;
        GFX.drawPixel(xPos , yPos, grain[i].colour);
    }
    GFX.end();
}