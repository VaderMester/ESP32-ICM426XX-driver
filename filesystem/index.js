var fibres = 0;
var tc = GFX.colorMake(255, 0, 0, 0);
var timec = GFX.colorMake(100, 255, 50, 0);
var monc = GFX.colorMake(255, 100, 255, 0);
var dayc = GFX.colorMake(255, 255, 0, 0);
var yearc = GFX.colorMake(0, 255, 255, 0);
var blink = false;
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

var gyro = new Array(6);

//var tzoffset = 2;

function fib(i) {
    return 0 == i ? 0 : 1 == i ? 1 : fib(i - 1) + fib(i - 2)
}

function test(tcolor, x, y) {
    for (i = 0; i < 20; i++) {
        fibres = fib(i);
        GFX.start();
        print(fibres);
        //console.log('fibres');
        GFX.clearBuf();
        GFX.printText(fibres, x, y, tcolor);
        GFX.end();
    }
}

function addZero(i) {
    if (i < 10) {
        i = "0" + i;
    }
    return i;
}

function drawTime1(mon, day, hours, min) {
    GFX.start();
    GFX.clearBuf();
    print(mon + " " + day + " " + hours + ":" + min);
    if (blink) {
        GFX.printText(hours + ":" + min, 0, 10, timec);
    } else {
        GFX.printText(hours + " " + min, 0, 10, timec);
    }
    GFX.printText(mon, 0, 22, monc);
    GFX.printText(day, 10, 30, dayc);
    GFX.end();
}

function drawTime2(years, mon, day, hours, min) {
    GFX.start();
    GFX.clearBuf();
    mon = mon.toString();
    day = day.toString();
    hours = hours.toString();
    min = min.toString();
    var xoffset = 0;
    print(years + "." + mon + "." + day + " " + hours + ":" + min);
    for (var i = 0; i < 6; i++) {
        xoffset = i * 32;
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
    }
    GFX.end();
}

GFX.clearAll();
for (i = 0; i < 20; i++) {
    GFX.start();
    GFX.fillScreen(-1);
    GFX.end();
}
GFX.clearAll();
GFX.setFont(0);
GFX.setFrameTime(1000);
while (true) {
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
        drawTime2(years, mon, day, hours, min);
        blink = !blink;
        gyro = cubeOS.getAccelGyro();
        print("ax: " + gyro[0] + "; ay: " + gyro[1] + "; az: " + gyro[2] + "; gx: " + gyro[3] + "; gy: " + gyro[4] + "; gz: " + gyro[5]);
}