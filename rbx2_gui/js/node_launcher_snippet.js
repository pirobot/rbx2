
// Launch and kill service variables
var launchProcessService;
var killProcessService;

launchProcessService = new ROSLIB.Service({
	ros : ros,
	name : '/node_launcher/launch_process',
	serviceType : 'rbx2_utils/LaunchProcess'
    });

killProcessService = new ROSLIB.Service({
	ros : ros,
	name : '/node_launcher/kill_process',
	serviceType : 'rbx2_utils/KillProcess'
    });


// First, we create a Service client with details of the service's
// name and service type.
var openniNode = new ROSLIB.Service({ ros : ros, name : '/node_launcher/launch_process', serviceType : 'rbx2_utils/LaunchProcess' });

// Then we create a Service Request. The object we pass in to
// ROSLIB.ServiceRequest matches the // fields defined in the
// rospy_tutorials AddTwoInts.srv file.
var request = new ROSLIB.ServiceRequest({ run_type : 'roslaunch', package_name: 'rbx2_vision', filename: 'openni_node.launch', params: '' });

// Finally, we call the service and get back the results in the
// callback. The result // is a ROSLIB.ServiceResponse object.
openniNode.callService(request, function(result) { console.log('Result for service call on ' + openniNode.name + ': ' + result.process_id)});
var param = new ROSLIB.Param({ ros : ros, name : param_ns + '/openni_node_process_id' }); param.set(result.process_id); });
