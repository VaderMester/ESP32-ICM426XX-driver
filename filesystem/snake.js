//Display
var WIDTH = 192-1;	//global WIDTH (coordinates are from 0-191)
var HEIGHT = 32-1;	//global Height (coordinates are from 0-31)
var FPS = 24;       //Actual frametime setting
var animFPS = 2;    //what the animation will actually do.
var currf = 0;
var workingFrames = Math.ceil(80/(1000/FPS));
var dummyFrames = (FPS/animFPS)-workingFrames;  //FPS and animFPS must be devisable with 0 modulus.

//dummyFrames variable gives the amount of frames between renders that will be spent with other stuff, but rendering.
//In our case, dummy frame times are used to read accelGyro values, to get the correct movements.
//workingFrames are given that we assume that it will take given ms of time for every game and image calculation to be completed.

var scoreOffset = 160;  //Score will be displayed on top Side, which starts at x:160px

var snakeboard = { width: WIDTH-64, height: HEIGHT }; //bottom and top sides of the cube are not used
var snakeStroke = 2;	//Width of snake body in pixels
var tailTransparency = 0.50;		//percentage of tail transparency

var startX = 31 + 16;
var startY = snakeboard.height / 2 - 2;

var red = GFX.colorMake(255, 0, 0, 0);
var lightgreen = GFX.colorMake(100, 255, 50, 0);
var magenta = GFX.colorMake(255, 100, 255, 0);
var yellow = GFX.colorMake(255, 255, 0, 0);
var cyan = GFX.colorMake(0, 255, 255, 0);
var myRED = GFX.colorMake(255, 0, 0, 0);
var myGREEN = GFX.colorMake(0, 255, 0, 0);
var myBLUE = GFX.colorMake(0, 0, 255, 0);
var myMAGENTA = magenta;
var myYELLOW = yellow;
var myCYAN = cyan;
var myBLACK = GFX.colorMake(0, 0, 0, 0);
var myWHITE = GFX.colorMake(255, 255, 255, 0);
var myORANGE = GFX.colorMake(235, 155, 52, 0);
//beggining of snake
var blink = false; //blink strobe for animation

function vector(x, y, z) {
    this.xAxis = x;
    this.yAxis = y;
    this.zAxis = z;
}

var accelGyro = new Array(6);
var accelVector = new vector();
var gyrotresholdP = 100;
var gyrotresholdN = -100;
var accelTreshold = 1000;

var LEFT = 1;
var RIGHT = 2;
var UP = 3;
var DN = 4;

var currentDirection = 0;
var shaken = false;

var snakeBodyColor = myRED;
var snakeHeadColor = myORANGE;
var snakeTailColor = GFX.colorMake(255, 0, 0, (255 * tailTransparency));
var foodcolor = myWHITE;

var board_border = 'black';
var board_background = "white";
var snake_col = 'lightblue';
var snake_border = 'darkblue';

function snakeObj(xpos, ypos, part, color) {
    this.x = xpos;
    this.y = ypos;
    this.part = part;
    this.color = color;	//Body part type = 1: head, 2: body, 3: tail
}

var snake = new Array();

function handleDummyFrames(dummyf) {
    var i = 0;
    var readcnt = 0;
    var dirstr = "  ";
    var prevAccel = {ax: accelGyro[0], ay: accelGyro[1], az: accelGyro[2]};
    while(i < dummyf-1) {
        GFX.start();
        accelGyro = cubeOS.getAccelGyro();
        print("ax: " + accelGyro[0] + "; ay: " + accelGyro[1] + "; az: " + accelGyro[2] + "; gx: " + accelGyro[3] + "; gy: " + accelGyro[4] + "; gz: " + accelGyro[5]);
        if(currentDirection == 0) {
            currentDirection = getDirection(snake[0].x, accelGyro[3], accelGyro[4], accelGyro[5]);
            readcnt++;
        }
        if(shaken == false) {
            shaken = detectShake(prevAccel[0], prevAccel[1], prevAccel[2], accelGyro[0], accelGyro[1], accelGyro[2], accelTreshold);
        }
        GFX.end();
        i++;
    }
    if(currentDirection == UP) dirstr = "UP";
    if(currentDirection == DN) dirstr = "UN";
    if(currentDirection == LEFT) dirstr = "LE";
    if(currentDirection == LEFT) dirstr = "RI";
    print("Direction detected: " + currentDirection + " (" + dirstr + ") after '" + readcnt + "' times");
    cubeOS.gc();
}

function detectShake(pax, pay, paz, ax, ay, az, threshold) {
    //Wait for Z acceleration to be greater than 0 (shake or turn upside down)
    if((pax - ax) >= threshold || (ax - pax) >= threshold || (pay - ay) >= threshold || (ay - pay) >= threshold || (paz - az) >= threshold || (az - paz) >= threshold)
    {
        print("!!!Shake detected!!!");
        return true;
    }
    return false;
}


function getDirection(snakex, gx, gy, gz)
{
    //Z change is panel independent
    if(gz>gyrotresholdP) return RIGHT;
    if(gz<gyrotresholdN) return LEFT;
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


function genSnake() {
    for (var i = 1; i < 4; i++) {
        var snakepart = new snakeObj(startX - (i * 2), startY, i, snakeBodyColor);
        snake.push(snakepart);
    }
    snake[0].color = snakeHeadColor;
    snake[2].color = snakeTailColor;
}	// now, snake array has 3 items, 0 is the head, last is the tail, and middle is the body part

var score = 0;
// True if changing direction
var changing_direction = false;
// Horizontal velocity
var food_x;
var food_y;
var dx = 3;
// Vertical velocity
var dy = 0;


// Start game
genSnake();
gen_food();
main();
gen_food();

// main function called repeatedly to keep the game running
function main() {
shaken = false;
    if (has_game_ended()) {
        var restarting = true;
        while(restarting)
        {
            handleDummyFrames(dummyFrames);
            if(shaken) restarting = false;
        }
        startGame();
    }
    handleDummyFrames(dummyFrames);
    print("Snake On");
    GFX.start();
    changing_direction = false;
    clear_board();
    drawFood();
    move_snake();
    drawSnake();
    GFX.end();
    print("Snake Off");
    // Repeat
    main();
}

// draw a border around the canvas
function clear_board() {
    //  Clear display
    GFX.clearBuf();
    // Draw a top and bottom "border" around the entire play area
    GFX.drawHLine(0, 0, snakeboard.width, myRED);
    GFX.drawHLine(0, snakeboard.height, snakeboard.width, red);
}

// Draw the snake on the canvas
function drawSnake() {
    // Draw each part
    snake.forEach(drawSnakePart)
}

function drawFood() {
    GFX.fillRoundRect(food_x, food_y, 3, 3, 1, foodcolor);
}

// Draw one snake part
function drawSnakePart(snakePart) {
    // Draw a "filled" rectangle to represent the snake part at the coordinates
    // the part is located
    GFX.fillRect(snakePart.x, snakePart.y, snakeStroke, snakeStroke, snakepart.color);
}

function has_game_ended() {
    for (var i = 4; i < snake.length; i++) {
        if (snake[i].x === snake[0].x && snake[i].y === snake[0].y) return true
    }
    //var hitLeftWall = snake[0].x < 0;
    //var hitRightWall = snake[0].x > snakeboard.width - 10;
    var hitToptWall = snake[0].y < 0;
    var hitBottomWall = snake[0].y > snakeboard.height - 10;
    return hitToptWall || hitBottomWall;
}

function random_food(min, max) {
    return Math.round(((Math.random() * (max - min) + min) / 10) * 10);
}

function gen_food() {
    // Generate a random number the food x-coordinate
    food_x = random_food(0, snakeboard.width - 2);
    // Generate a random number for the food y-coordinate
    food_y = random_food(0, snakeboard.height - 2);
    // if the new food location is where the snake currently is, generate a new food location
    snake.forEach(function has_snake_eaten_food(part) {
        var has_eaten = part.x == food_x && part.y == food_y;
        if (has_eaten) gen_food();
    });
    //The ugly and stinky shit below supposed to draw helper direction indicators to the user to find the food with the snake
    //This is usefull, becuase the food can be generated to a cube side not wisible by the player
}

function change_direction(event) {
    // Prevent the snake from reversing
    if (changing_direction) return;
    changing_direction = true;
    var goingUp = dy === -snakeStroke;
    var goingDown = dy === snakeStroke;
    var goingRight = dx === snakeStroke;
    var goingLeft = dx === -snakeStroke;

    if (currentDirection === LEFT && !goingRight) {
        dx = -3;
        dy = 0;
    }
    if (currentDirection === UP && !goingDown) {
        dx = 0;
        dy = -3;
    }
    if (currentDirection === RIGHT && !goingLeft) {
        dx = 3;
        dy = 0;
    }
    if (currentDirection === DN && !goingUp) {
        dx = 0;
        dy = 3;
    }
}

function scoreToString(score) {
    var scorestr = "000" + score;
    scorestr = scorestr.substr(scorestr.length - 4);
    return scorestr;
}

function move_snake() {
    // Create the new Snake's head
    var head = new snakeObj(snake[0].x + dx, snake[0].y + dy, 0, snakeHeadColor);
    // Add the new head to the beginning of snake body
    snake.unshift(head);
    var has_eaten_food = snake[0].x === food_x && snake[0].y === food_y;
    if (has_eaten_food) {
        // Increase score
        score += 5;
        // Display score on screen
        GFX.setFont(0);
        GFX.printText("s", 160 - 0, 7, myGREEN);
        GFX.printText("c", 160 + 6, 7, myGREEN);
        GFX.printText("o", 160 + 12, 7, myGREEN);
        GFX.printText("r", 160 + 18, 7, myGREEN);
        GFX.printText("e", 160 + 24, 7, myGREEN);
        GFX.printText(scoreToString(score), 160, 20, myCYAN);
        // Generate new food location
        gen_food();
    } else {
        // Remove the last part of snake body
        snake.pop();
        snake[snake.length-1].part = 3;
        snake[snake.length-1].color = snakeTailColor;
    }
    snake.forEach(function (part) {
        if(part.x > 127 ) part.x -= (WIDTH+1);
        if(part.x < 0 ) part.x += (WIDTH+1);
    });
    if(snake[0].x>0 && snake[0].x<32) {
        //Panel 0
        if (food_x > 31 && food_x < (snake[0].x + 64)) {
            GFX.drawVLine(31, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(30, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(29, 6, 20, GFX.colorMake(100, 255, 50, 127));
        } else if (food_x > (snake[0].x + 63)) {
            GFX.drawVLine(0, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(1, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(2, 6, 20, GFX.colorMake(100, 255, 50, 127));
        }
    }
    if(snake[0].x>31 && snake[0].x<64) {
        //Panel 1
        if (food_x > 63 && food_x < (snake[0].x + 64)) {
            GFX.drawVLine(63, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(62, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(61, 6, 20, GFX.colorMake(100, 255, 50, 127));
        } else if (food_x > (snake[0].x + 63) || food_x < 32) {
            GFX.drawVLine(32, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(33, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(34, 6, 20, GFX.colorMake(100, 255, 50, 127));
        }
    }
    if(snake[0].x>63 && snake[0].x<96) {
        //Panel 2
        if (food_x > 95 && food_x < (snake[0].x + 64-128)) {
            GFX.drawVLine(95, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(94, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(93, 6, 20, GFX.colorMake(100, 255, 50, 127));
        } else if (food_x > (snake[0].x + 63-128) || food_x < 64) {
            GFX.drawVLine(64, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(65, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(66, 6, 20, GFX.colorMake(100, 255, 50, 127));
        }
    }
    if(snake[0].x>95 && snake[0].x<128) {
        //Panel 3
        //Panel 2
        if (food_x >= 0 && food_x < (snake[0].x + 64-128)) {
            GFX.drawVLine(127, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(126, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(125, 6, 20, GFX.colorMake(100, 255, 50, 127));
        } else if (food_x > (snake[0].x + 63-128) && food_x < 96) {
            GFX.drawVLine(0, 2, 28, GFX.colorMake(100, 255, 50, 0));
            GFX.drawVLine(1, 4, 24, GFX.colorMake(100, 255, 50, 64));
            GFX.drawVLine(2, 6, 20, GFX.colorMake(100, 255, 50, 127));
        }
    }
}

function startGame() {
    var wait = 5;
    while(wait >= 0) {
        var currf = 0;
        while(currf < FPS) {
            GFX.start();
            GFX.clearBuf();
            var arrowlen = 1+(25/FPS)*currf;
            //draw arrow Panel 0;
            var arrowstart = (32-arrowlen)/2;
            GFX.drawRect(arrowstart, 15, arrowlen, 2, red);
            GFX.drawLine(arrowstart+arrowlen, 15, arrowstart+arrowlen-7, 15-7, red);
            GFX.drawLine(arrowstart+arrowlen, 16, arrowstart+arrowlen-7, 15+7, red);
            //draw arrow Panel 2
            GFX.drawRect(arrowstart+64, 15, arrowlen, 2, red);
            GFX.drawLine(arrowstart+64, 15, arrowstart+7, 15-7, red);
            GFX.drawLine(arrowstart+64, 16, arrowstart+7, 15+7, red);
            //draw arrow Panel 3
            GFX.drawRect(arrowstart+96, 15, arrowlen, 2, red);
            GFX.drawLine(arrowstart+96, 15, arrowstart+7, 15-7, red);
            GFX.drawLine(arrowstart+96, 16, arrowstart+7, 15+7, red);

            GFX.setFont(1);
            GFX.printText(wait, 10, 20, myORANGE);
            GFX.setFont(0);
            GFX.printText("Start", 159, 15, myWHITE);
            GFX.end();
            currf++;
        }
        wait--;
    }
    score = 0;
    GFX.clearBuf();
    genSnake();
}
//********END OF SNAKE	