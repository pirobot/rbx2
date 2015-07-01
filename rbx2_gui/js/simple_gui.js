/*  simple_gui.js - Version 1.0 2013-09-29

    A simple HTML5/rosbridge script to control and monitor a ROS robot

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
 */

// Set the rosbridge and mjpeg_server port
var rosbridgePort = "9090";
var mjpegPort = "8080";

// Get the current hostname
thisHostName = document.location.hostname;

// Set the rosbridge and mjpeg server hostname accordingly
var rosbridgeHost = thisHostName;
var mjpegHost = thisHostName;

// Build the websocket URL to the rosbride server
var serverURL = "ws://" + rosbridgeHost + ":" + rosbridgePort;

// The mjpeg client object that will be defined later
var mjpegViewer;

// The default video topic (can be set in the rosbridge launch file)
var videoTopic = "/camera/rgb/image_raw";

// The mjpeg video quality (percent)
var videoQuality = 50;

// A varible to hold the chatter topic and publisher
var chatterTopic;

// A variable to hold the chatter message
var chatterMsg;

// The ROS namespace containing parameters for this script
var param_ns = '/robot_gui';

// Are we on a touch device?
var isTouchDevice = 'ontouchstart' in document.documentElement;

// A flag to indicate when the Shift key is being depressed
// The Shift key is used as a "dead man's switch" when using the mouse
// to control the motion of the robot.
var shiftKey = false;

// The topic on which to publish Twist messages
var cmdVelTopic = "/cmd_vel";

// Default linear and angular speed
var defaultLinearSpeed = 0.2;
var defaultAngularSpeed = 1.0;

// Current maximum linear and angular speeds (can be overriden in rosbridge.launch)
var maxLinearSpeed = defaultLinearSpeed;
var maxAngularSpeed = defaultAngularSpeed;

// Minimum linear and angular speed
var minLinearSpeed = 0.05;
var minAngularSpeed = 0.1;

// How much to increment speeds with each key press
var vx_click_increment = 0.05;
var vz_click_increment = 0.1;

// Current desired linear and angular speed
var vx = 0.0;
var vz = 0.0;

// A handle for the publisher timer
var pubHandle = null;

// A handle for the stop timer
var stopHandle = null;

// The rate in Hz for the main ROS publisher loop
var rate = 5;

// Get the current window width and height
var windowWidth = this.window.innerWidth;
var windowHeight = this.window.innerHeight;

// Set the video width to 1/2 of the window width and scale the height
// appropriately.
var videoWidth = Math.round(windowWidth / 2.0);
var videoHeight = Math.round(videoWidth * 240 / 320);

//The main ROS object
var ros = new ROSLIB.Ros();

// Connect to ROS
function init_ros() {
	ros.connect(serverURL);
	
	// Set the rosbridge host and port values in the form
	document.getElementById("rosbridgeHost").value = rosbridgeHost;
	document.getElementById("rosbridgePort").value = rosbridgePort;
}

// If there is an error on the back end, an 'error' emit will be emitted.
ros.on('error', function(error) {
	console.log("Rosbridge Error: " + error);
});

// Wait until a connection is made before continuing
ros.on('connection', function() {
	console.log('Rosbridge connected.');

	// Create a Param object for the video topic
	var videoTopicParam = new ROSLIB.Param({
		ros : ros,
		name : param_ns + '/videoTopic'
	});

	videoTopicParam.get(function(value) {
	    if (value != null) {
		videoTopic = value;
	    }
		
	    // Create the video viewer
	    if (!mjpegViewer) {
		mjpegViewer = new MJPEGCANVAS.Viewer({
		    divID : 'videoCanvas',
		    host : mjpegHost,
		    port : mjpegPort,
		    width : videoWidth,
		    height : videoHeight,
		    quality : videoQuality,
		    topic : videoTopic
		});
	    }
	});

	// Create a Param object for the max linear speed
	var maxLinearSpeedParam = new ROSLIB.Param({
		ros : ros,
		name : param_ns + '/maxLinearSpeed'
	});

	// Get the value of the max linear speed paramater
	maxLinearSpeedParam.get(function(value) {
		if (value != null) {
			maxLinearSpeed = value;
						
			// Update the value on the GUI
			var element = document.getElementById('maxLinearSpeed');
			element.setAttribute("max", maxLinearSpeed);
			element.setAttribute("value", defaultLinearSpeed);

			writeStatusMessage('maxLinearSpeedDisplay', maxLinearSpeed.toFixed(2));
		}
	});

	// Create a Param object for the max angular speed
	var maxAngularSpeedParam = new ROSLIB.Param({
		ros : ros,
		name : param_ns + '/maxAngularSpeed'
	});

	// Get the value of the max angular speed paramater
	maxAngularSpeedParam.get(function(value) {
		if (value != null) {
			maxAngularSpeed = value;
			
			// Update the value on the GUI
			var element = document.getElementById('maxAngularSpeed');
			element.setAttribute("max", maxAngularSpeed);
			element.setAttribute("value", defaultAngularSpeed);

			writeStatusMessage('maxAngularSpeedDisplay', maxAngularSpeed.toFixed(2));
		}
	});
	
	// Create the chatter topic and publisher
	chatterTopic = new ROSLIB.Topic({
		ros : ros,
		name : '/chatter',
		messageType : 'std_msgs/String',
	});
	
	// Create the chatter message
	var message = document.getElementById('chatterMessage');
	chatterMsg = new ROSLIB.Message({
		data : message.value
	});

	document.addEventListener('keydown', function(e) {
		if (e.shiftKey)
			shiftKey = true;
		else
			shiftKey = false;
		setSpeed(e.keyCode);
	}, true);

	document.addEventListener('keyup', function(e) {
		if (!e.shiftKey) {
			shiftKey = false;
			stopRobot();
		}
	}, true);	

	// Display a line of instructions on how to move the robot
	if (isTouchDevice) {
		// Set the Nav instructions appropriately for touch
		var navLabel = document.getElementById("navInstructions");
		navLabel.innerHTML = "Tap an arrow to move the robot";
		
		// Hide the publish/subscribe rows on touch devices
		var pubSubBlock = document.getElementById("pubSubBlock");
		pubSubBlock.style.visibility = 'hidden';
	}
	else { 
		// Set the Nav instructions appropriately for mousing
		var navLabel = document.getElementById("navInstructions");
		navLabel.innerHTML = "Hold down SHIFT Key when clicking arrows";
	}

	// Start the publisher loop
	console.log("Starting publishers");
	pubHandle = setInterval(refreshPublishers, 1000 / rate);
});

function toggleChatter() {
	var pubChatterOn = document.getElementById('chatterToggle').checked;
	if (pubChatterOn) chatterTopic.advertise();
	else chatterTopic.unadvertise();
}

function updateChatterMsg(msg) {
	chatterMsg.data = msg;
}

function refreshPublishers() {
	// Keep the /cmd_vel messages alive
	pubCmdVel();
		
	if (chatterTopic.isAdvertised)
		chatterTopic.publish(chatterMsg);
}

var cmdVelPub = new ROSLIB.Topic({
	ros : ros,
	name : cmdVelTopic,
	messageType : 'geometry_msgs/Twist'
});

function pubCmdVel() {
	vx = Math.min(Math.abs(vx), maxLinearSpeed) * sign(vx);
	vz = Math.min(Math.abs(vz), maxAngularSpeed) * sign(vz);

	if (isNaN(vx) || isNaN(vz)) {
		vx = 0;
		vz = 0;
	}

	var cmdVelMsg = new ROSLIB.Message({
		linear : {
			x : vx,
			y : 0.0,
			z : 0.0
		},
		angular : {
			x : 0.0,
			y : 0.0,
			z : vz
		}
	});
	
	var statusMessage = "vx: " + vx.toFixed(2) + " vz: " + vz.toFixed(2);
	writeStatusMessage('cmdVelStatusMessage', statusMessage);

	cmdVelPub.publish(cmdVelMsg);
}

// Speed control using the arrow keys or icons
function setSpeed(code) {
	// Stop if the deadman key (Shift) is not depressed
	if (!shiftKey && !isTouchDevice) {
		stopRobot();
		return;
	}

	// Use space bar as an emergency stop
	if (code == 32) {
		vx = 0;
		vz = 0;
	}
	// Left arrow
	else if (code == "left" || code == 37) {
		vz += vz_click_increment;
	}
	// Up arrow
	else if (code == 'forward' || code == 38) {
		vx += vx_click_increment;
	}
	// Right arrow
	else if (code == 'right' || code == 39) {
		vz -= vz_click_increment;
	}
	// Down arrow
	else if (code == 'backward' || code == 40) {
		vx -= vx_click_increment;
	}
}

function stopRobot() {
	vx = vz = 0;
	var statusMessage = "vx: " + vx.toFixed(2) + " vz: " + vz.toFixed(2);
	writeStatusMessage('cmdVelStatusMessage', statusMessage);
	pubCmdVel();
}

function timedStopRobot() {
    stopHandle = setTimeout(function() { stopRobot() }, 1000);
}

function clearTimedStop() {
	clearTimeout(stopHandle);
}

function subChatter() {
	var subscribe = document.getElementById('chatterSub').checked;
	var chatterData = document.getElementById('chatterData');
	var listener = chatterTopic;

	if (subscribe) {
		console.log('Subscribed to ' + listener.name);
		listener.subscribe(function(msg) {
			chatterData.value = msg.data;
		});
	} else {
		listener.unsubscribe();
		console.log('Unsubscribed from ' + listener.name);
	}
}

function setROSParam() {
	var paramName = document.getElementById('setParamName');
	var paramValue = document.getElementById('setParamValue');
	var param = new ROSLIB.Param({
		ros : ros,
		name : paramName.value
	});
	
    if (isNumeric(paramValue.value)) {
		param.set(parseFloat(paramValue.value));
    }
    else {
    	param.set(paramValue.value);
    }
}

function getROSParam() {
	var paramName = document.getElementById('getParamName');
	var paramValue = document.getElementById('getParamValue');
	var param = new ROSLIB.Param({
		ros : ros,
		name : paramName.value
	});
	param.get(function(value) {
		paramValue.value = value;
	});
}

function setMaxLinearSpeed(speed) {
	maxLinearSpeed = speed;
	console.log("Max linear speed set to: " + maxLinearSpeed);
}

function getMaxLinearSpeed() {
	return maxLinearSpeed;
}

function setMaxAngularSpeed(speed) {
	maxAngularSpeed = speed;
	console.log("Max angular speed set to: " + maxAngularSpeed);
}

function connectDisconnect() {
	var connect = document.getElementById('connectROS').checked;

	if (connect)
		connectServer();
	else
		disconnectServer();
}

function disconnectServer() {
	console.log("Disconnecting from ROS.");
	mjpegViewer.changeStream();
	ros.close();
}

function connectServer() {
	rosbridgeHost = document.getElementById("rosbridgeHost").value;
	rosbridgePort = document.getElementById("rosbridgePort").value;
	serverURL = "ws://" + rosbridgeHost + ":" + rosbridgePort;
	mjpegViewer.changeStream(videoTopic);
	try {
		ros.connect(serverURL);
		console.log("Connected to ROS.");
	} catch (error) {
		console.write(error);
	}
}

function writeStatusMessage(field, message, color) {
	color = typeof color !== 'undefined' ? color : "#006600";
	element = document.getElementById(field);
	element.innerHTML = message;
	element.style.font = "18pt Calibri";
	element.style.color = color;
}

function sign(x) {
	if (x < 0) {
		return -1;
	}
	if (x > 0) {
		return 1;
	}
	return 0;
}

function isNumeric(n) {
  return !isNaN(parseFloat(n)) && isFinite(n);
}
