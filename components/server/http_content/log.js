document.getElementById("websocketstatus").innerHTML = "Not connected";
var websocket = new WebSocket('ws://' + location.hostname + '/');

function connect() {
	websocket = new WebSocket('ws://' + location.hostname + '/');
};

var sendbutton = document.getElementById("send");
var counter = 0;
var list = document.getElementById('list');
var scrl = true;
var scrolltimeout;
var scrldelay = 10000;

sendbutton.onclick = function () {
	websocket.send(document.getElementById("websocketcommand").value);
}

function ab2str(buf) {
	return new Uint8Array(buf).toString();
}

function keepalive() {
	var msg = "#" + counter;
	websocket.send(msg);
}

function pauseScroll() {
	scrl = false;
	console.log("Scroll status" + scrl);
	if (typeof scrolltimeout == "undefined") {
		scrolltimeout = setTimeout(function () { scrl = true }, scrldelay);
	} else {
		clearTimeout(scrolltimeout);
		scrolltimeout = setTimeout(function () { scrl = true }, scrldelay);
	}
}

function isScrolledIntoView(elem) {
	var y = elem.scrollTop;
	console.log("y: " + y);
	var h = elem.scrollHeight - 620;
	console.log("h: " + h);
	return ((y >= h));
}

websocket.onopen = function (evt) {
	document.getElementById("websocketstatus").innerHTML = "Connected!";
	keepalive(counter);
	setInterval(keepalive, 1000);
}

websocket.onmessage = function (evt) {
	var list = document.getElementById('list');
	var count = 0;
	var item = document.createElement('li');
	if (typeof evt.data === 'string') {
		counter++;
		var timestamp = new Date();
		var millis = timestamp.getMilliseconds(); //pad millicesonds with 0s, to 3 digits
		millis = "00" + millis;
		millis = millis.substr(millis.length - 3);
		var tmst = timestamp.toLocaleTimeString('hu-HU') + "." + millis + " -";
		//var instring = evt.data.substr(0, evt.data.length-1);
		var instring = new String(evt.data);
		//item.innerHTML = instring.split('\n').map(s => `${tmst} ${s}`).join('\n');
		item.innerHTML = instring.split('\n').map(s => `${tmst} ${s}`).join('\n');
	} else {
		var reader = new FileReader();
		reader.readAsArrayBuffer(evt.data);
		reader.addEventListener("loadend", function (e) {
			item.innerHTML = counter + " Binary data received in " + e.target.result.byteLength + " long octet stream: " + ab2str(e.target.result);
		});
	}
	if (isScrolledIntoView(list) == true) {
		scrl = true;
		if (typeof scrolltimeout !== "undefined") {
			clearTimeout(scrolltimeout);
		}
	}
	list.appendChild(item);
	if (scrl == true) {
		list.scrollTop = list.scrollHeight - list.clientHeight;
	}
}

websocket.onclose = function (evt) {
	document.getElementById("websocketstatus").innerHTML = "Closed";
	setTimeout(function() {
		connect();
	  }, 1000);
}

websocket.onerror = function (evt) {
	document.getElementById("websocketstatus").innerHTML = "Error";
}

