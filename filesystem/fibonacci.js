var fibres = 0;

function fib(i) {
    return 0==i?0:1==i?1:fib(i-1)+fib(i-2)
}

function test(tcolor, x, y) {
    for(i=0; i < 20; i++) {
        fibres = fib(i);
        GFX.start();
        print(fibres);
        //console.log('fibres');
        GFX.clearBuf();
        GFX.printText(fibres.toString(), x, y, tcolor);
        GFX.end();
    }
}

GFX.clearAll();
for(i=0; i<20; i++) {
    GFX.start();
    GFX.fillScreen(-1);
    GFX.end();
}
GFX.clearAll();
var tc = GFX.colorMake(255, 0, 0, 0);
var x = 1;
var y = 10;
GFX.setFont(0);
GFX.setFrameTime(40);
print("Running Fibonacci.js");
//console.log('Running Fibonacci.js');
while(true) {
    test(tc, x, y);
}