/*
 * Master loop of ESP32-Duktape
 * This is the loop function that is called as often as possible within the ESP32-Duktape
 * environment.  It is primarily responsible for:
 * 
 * * Checking to see if timers have expired.
 * * Polling network I/O to see if network actions are needed.
 */
/* globals _sockets, OS, log, Buffer, require, ESP32, _timers, module, DUKF */
/**
 * Primary loop that processes events.
 * @returns
 */
function loop() {
	// Timer examination
	//
	// Handle a timer having expired ... we have a global variable called _timers.  This variable
	// contains an array called timerEntries.  Each entry in timerEntries contains:
	// {
	//    id:       The ID of the timer.
	//    callback: The function to be invoked on the callback.
	//    fire:     The time when the timer expires
	//    interval: 0 for a single timeout and a value > 0 for an interval timer.
	// }
	//
	
	if (_timers.timerEntries.length > 0 && new Date().getTime() >= _timers.timerEntries[0].fire ) {
		//log("Processing timer fired for id: " + _timers.timerEntries[0].id);
		var timerCallback = _timers.timerEntries[0].callback;
		if (_timers.timerEntries[0].interval > 0) {
			_timers.timerEntries[0].fire = new Date().getTime() + _timers.timerEntries[0].interval;
			_timers.sort();
		} else {
			_timers.timerEntries.splice(0, 1);
		}
		timerCallback();
	} // End of check timers.
	DUKF.gc();
} // loop

log("Major function \"loop()\" registered");

module.exports = loop;