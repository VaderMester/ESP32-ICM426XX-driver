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

$(function() {
	const CUBEOS_IP   = location.hostname;
	const CUBEOS_PORT = 80;
	var fileNameDeferred = null;
	const FILE_DIRECTORY = 2; // Type of file that is a directory
	const FILE_REGULAR   = 1; // Type of file that is a regular file
	//var bleClient = null;
	//var bleServerNode = null;
	
	function getFileNameDialog() {
		fileNameDeferred = jQuery.Deferred();
		$("#fileNameText").val("");
		$("#fileNameDialog").dialog("open");
		return fileNameDeferred;
	}
	
	
	function doRest(path, callback, obj) {
		obj.url      = "http://" + CUBEOS_IP + ":" + CUBEOS_PORT + path;
		obj.dataType = "json";
		obj.success  = function(data, textStatus, jqXHR) {
			if (callback) {
				callback(data);
			}
		};
		obj.error    = function(jqXHR, textStatus, errorThrown) {
			console.log("AJAX error");
			debugger;
		};
		jQuery.ajax(obj);
	} // doRest
	
	function getData(path, callback) {
		doRest(path, callback, {
			method   : "GET"
		});
	} // getData
	
	function deleteData(path, callback, _data) {
		doRest(path, callback, {
			method   : "DELETE",
			data: _data
		});
	} // deleteData

	function postData(path, callback, _data) {
		doRest(path, callback, {
			method   : "POST",
			data: _data
		});
	} // postData
	
	
	function addFileDirectory(childrenArray, fileEntries) {
/*
 * File entries is an array of objects
 * path: Full path name
 * name: name of file/directory
 * directory: true/false (is this a directory)
 * dir: [child path entries]
 */
		for (var i=0; i<fileEntries.length; i++) {
			var newChild = {
				text: fileEntries[i].name,
				icon: fileEntries[i].directory?"jstree-folder":"jstree-file",
				data: {
					path: fileEntries[i].path,
					directory: fileEntries[i].directory
				}
			}
			if (fileEntries[i].directory) {
				var newChildren = [];
				addFileDirectory(newChildren, fileEntries[i].dir);
				newChild.children = newChildren;
			}
			childrenArray.push(newChild);
		}
	} // addFileDirectory
	
	
	function buildFileSystemTree(path) {  // TODO refactor to match new regex
		if (path.length == 0) {
			return;
		}
		console.log("Get the directory at: " + path);
		getData("/cubeOS/FILE" + path, function(data) {
			// We now have data!
			var root = {
				text: path,
				state: {
					opened: true
				},
				icon: "jstree-folder",
				children: [],
				data: {
					path: path,
					directory: true
				}
			}
			addFileDirectory(root.children, data.dir);
			$('#fileTree').jstree(true).settings.core.data = root;
			$("#fileTree").jstree(true).refresh();
		});
	} // buildFileSystemTree


	function typeToString(type) {
		switch (type) {
		case 0x00:
			return "App";
		case 0x01:
			return "Data";
		default:
			return type.toString();
		}
	} // typeToString

	
	function subTypeToString(subtype) {
		switch (subtype) {
		case 0x00:
			return "Factory";
		case 0x01:
			return "Phy";
		case 0x02:
			return "NVS";
		case 0x03:
			return "Coredump";
		case 0x80:
			return "ESP HTTPD";
		case 0x81:
			return "FAT";
		case 0x82:
			return "SPIFFS";
		default:
			return subtype.toString();
		}
	} // subTypeToString

	
	console.log("cubeOS Manager loading");
	$("#myTabs").tabs();
	$("#systemTabs").tabs();

	$("#fileSystemRefreshButton").button().click(function() {
		var path = $("#rootPathText").val().trim();
		if (path.length == 0) {
			return;
		}
		buildFileSystemTree(path);
	});
	
	$('#fileTree').jstree({
		plugins: ["contextmenu"],
		contextmenu: {
			items: function(node) {
				var n = $('#fileTree').jstree(true).get_node(node);
				var items = {};
				if (n.data.directory) {
					items.createDir = {
			   		label: "Create",
			   		action: function(x) {
			   			var n= $('#fileTree').jstree(true).get_node(x.reference);
			   			console.log("Create a directory as a child of " + n.data.path);
			   			getFileNameDialog().done(function(fileName) {
				   			postData("/cubeOS/FILE" + n.data.path + "/" + fileName + "?directory=true");
			   			});
			   			//debugger;
			   		}
			   	}
				}
				else {
					items.downloadFile = {
			   		label: "Download",
			   		action: function(x) {
			   			var n = $('#fileTree').jstree(true).get_node(x.reference);
			   			console.log("Download the file called " + n.data.path);
			   			// Form a web socket to download the data ...
			   			var ws = new WebSocket("ws://" + CUBEOS_IP + ":" + CUBEOS_PORT);
			   			ws.onopen = function() {
			   				console.log("Web Socket now open!  Sending command ...");
			   				ws.send(JSON.stringify({
			   					command: "getfile",
			   					path: n.data.path
			   				}));
			   				ws.close();
			   			} 
			   		}
					};
					items.deleteFile = {
			   		label: "Delete",
			   		action: function(x) {
			   			var n= $('#fileTree').jstree(true).get_node(x.reference);
			   			console.log("Delete the file called " + n.data.path);
			   			deleteData("/cubeOS/FILE" + n.data.path);
			   		}
			   	};
				}
			   return items;
			}
		}
	});
	$("submit").on('click', function(){
		evt.preventdefault();/*
		postData("/cubeOS/FILE", 
		function(data){
			
			debugger;
		}, 
		{
			
		});*/
		console.log('submit');
	});
/*	
	$("#fileUploadForm").fileupload({
		add: function(e, data) {

			var selected = $("#fileTree").jstree("get_selected");

			if (selected.length > 0) {
				var node = $("#fileTree").jstree(true).get_node(selected[0]);
				data.formData = { path: node.data.path };
				data.submit();
			}
			//debugger;
		},
		done: function(e, data) {
			var path = $("#rootPathText").val().trim();
			if (path.length == 0) {
				return;
			}
			buildFileSystemTree(path);	
		}
	});
	*/
	
	$("#fileNameDialog").dialog({
		autoOpen: false,
		modal: true,
		title: "File name",
		width: 305,
		buttons: [
			{
				text: "OK",
				click: function() {
					fileNameDeferred.resolve($("#fileNameText").val());
					$("#fileNameDialog").dialog("close");
				}
			}
		]
	});
});
