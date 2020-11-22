var FPS = 24;       //Actual frametime setting
var animFPS = 2;    //what the animation will actually do.
var currf = 0;
//var workingFrames = Math.ceil(80/(1000/FPS));
var workingFrames = 0;
var dummyFrames = (FPS/animFPS)-workingFrames;  //FPS and animFPS must be devisable with 0 modulus.

//dummyFrames variable gives the amount of frames between renders that will be spent with other stuff, but rendering.
//In our case, dummy frame times are used to read accelGyro values, to get the correct movements.
//workingFrames are given that we assume that it will take given ms of time for every game and image calculation to be completed.

//beggining of snake
var blink = false; //blink strobe for animation

function vector(x, y, z) {
    this.xAxis = x;
    this.yAxis = y;
    this.zAxis = z;
}

var accelGyro = new Array(6);
var accelVector = new vector();
var gyrotresholdP = 200;
var gyrotresholdN = gyrotresholdP*-1;
var accelTreshold = 160;

var LEFT = 1;
var RIGHT = 2;
var UP = 3;
var DN = 4;

var currentDirection = 0;
var shaken = false;
function handleDummyFrames(dummyf) {
    shaken = false;
    var i = 0;
    var readcnt = 0;
    var dirstr = "  ";
    var prevAccel = {ax: accelGyro[0], ay: accelGyro[1], az: accelGyro[2]};
    while(i < dummyf) {
        GFX.start();
        accelGyro = Motion.getAccelGyro();
        //print("ax: " + accelGyro[0] + "; ay: " + accelGyro[1] + "; az: " + accelGyro[2] + "; gx: " + accelGyro[3] + "; gy: " + accelGyro[4] + "; gz: " + accelGyro[5]);
        if(currentDirection == 0) {
            currentDirection = getDirection(10, gyrotresholdP, gyrotresholdP);
            readcnt++;
        }
        if(shaken == false) {
            //print("shake meh");
            shaken = detectShake(prevAccel.ax, prevAccel.ay, prevAccel.az, accelGyro[0], accelGyro[1], accelGyro[2], accelTreshold);
        }
        GFX.end();
        i++;
    }
    if(currentDirection === UP) dirstr = "UP";
    if(currentDirection === DN) dirstr = "DN";
    if(currentDirection === LEFT) dirstr = "LE";
    if(currentDirection === RIGHT) dirstr = "RI";
    print("Direction detected: " + currentDirection + " (" + dirstr + ") after '" + readcnt + "' times");
    cubeOS.gc();
}

function detectShake(pax, pay, paz, ax, ay, az, threshold) {
    //Wait for Z acceleration to be greater than 0 (shake or turn upside down)
    //print((pax-ax) + " " + (ax - pax) + " " + (pay-ay) + " " + (ay - pay) + " " + (paz-az) + " " + (az - paz));
    if(((pax - ax) >= threshold) || ((ax - pax) >= threshold) || ((pay - ay) >= threshold) || ((ay - pay) >= threshold) || ((paz - az) >= threshold) || ((az - paz) >= threshold))
    {
        print("!!!Shake detected!!!");
        return true;
    } else {
    return false;
    }
}

function getDirection(snakex, thresholdP, thresholdN) {
    var gyrodir = Motion.getGyroDir(0, thresholdP, thresholdN);
    //Z change is panel independent
    if (snakex < 32) {
        //Panel 0
        switch (gyrodir) {
            case 0:
                return 0;
            case -1:
                return UP;
            case 1:
                return DN;
            case -3:
                return LEFT;
            case 3:
                return RIGHT;
            default:
                return 0;
        }
    } else if (snakex < 64) {
        //Panel 1
        switch (gyrodir) {
            case 0:
                return 0;
            case -2:
                return UP;
            case 2:
                return DN;
            case -3:
                return LEFT;
            case 3:
                return RIGHT;
            default:
                return 0;
        }
    } else if (snakex < 96) {
        //Panel 2
        switch (gyrodir) {
            case 0:
                return 0;
            case 1:
                return UP;
            case -1:
                return DN;
            case -3:
                return LEFT;
            case 3:
                return RIGHT;
            default:
                return 0;
        }
    } else if (snakex < 128) {
        //Panel 3
        switch (gyrodir) {
            case 0:
                return 0;
            case 2:
                return UP;
            case -2:
                return DN;
            case -3:
                return LEFT;
            case 3:
                return RIGHT;
            default:
                return 0;
        }
    } else if (snakex < 160) {
        switch (gyrodir) {
            case 0:
                return 0;
            case 1:
                return UP;
            case -1:
                return DN;
            case 3:
                return LEFT;
            case -3:
                return RIGHT;
            default:
                return 0;
        }
    } else {
        switch (gyrodir) {
            case 0:
                return 0;
            case -1:
                return UP;
            case 1:
                return DN;
            case -3:
                return LEFT;
            case 3:
                return RIGHT;
            default:
                return 0;
        }
    }
}

/*
function getDirection(snakex, gx, gy, gz)
{
    //Z change is panel independent
    if(gz>gyrotresholdP) return LEFT;
    if(gz<gyrotresholdN) return RIGHT;
    if(snakex>0 && snakex<32) {
        //Panel 0
        if(gx>gyrotresholdP) return DN;
        if(gx<gyrotresholdN) return UP;
    }
    if(snakex>31 && snakex<64) {
        //Panel 1
        if(gy>gyrotresholdP) return DN;
        if(gy<gyrotresholdN) return UP;
    }
    if(snakex>63 && snakex<96) {
        //Panel 2
        if(gx>gyrotresholdP) return UP;
        if(gx<gyrotresholdN) return DN;
    }
    if(snakex>95 && snakex<128) {
        //Panel 3
        if(gy>gyrotresholdP) return UP;
        if(gy<gyrotresholdN) return DN;
    }
    return 0;
}
*/
var cycle = 0;
function main() {
shaken = false;
currentDirection = false;
handleDummyFrames(dummyFrames);
cycle++;
print("******** CYCLE: " + cycle + " **********");
main();
}

print("******************Starting AccelGyro Test!********************");
main();