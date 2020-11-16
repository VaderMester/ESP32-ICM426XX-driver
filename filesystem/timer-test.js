/*
 *  Test using timers and intervals.
 */
var i;
var counters = [];
var ntimers = 0;

function addTimer(interval) {
    ntimers++;
    setTimeout(function () {
        ntimers--;
    }, interval);
}

function main() {

    print('set interval timer');
    var intervalTimer = setInterval(function () {
        print(new Date().toISOString() + ': timers pending: ' + ntimers);
        if (ntimers === 0) {
            clearInterval(intervalTimer);
        }
    }, 500);

    /* Here the inserts take a lot of time because the underlying timer manager
     * data structure has O(n) insertion performance.
     */
    print('launch timers');
    for (i = 0; i < 4000; i++) {
        // Math.exp(0)...Math.exp(8) is an uneven distribution between 1...~2980.
        addTimer(2000);
    }
    print('timers launched');
}

try {
    main();
} catch (e) {
    print(e.stack || e);
}
