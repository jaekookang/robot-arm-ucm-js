/*
 * 2018-12-23 Jaekoo Kang
 *
 * Note:
 * - To run locally, use `livereload` on Terminal.
 * - After importing lalolib, all the basic math functions are overriden 
 *   if their names overlap (eg, rand) -> see LALOLib.
 * 
 * References:
 * - PalinDrome555: http://jsfiddle.net/PalinDrome555/65v75m7j/3
 * - Plotly, restyle: https://codepen.io/etpinard/pen/kXGYyQ
 * - Plotly, aspect: https://plot.ly/javascript/3d-axes
 * - Plotly, meshgrid: https://plot.ly/javascript/3d-mesh
 * - LALOLib: http://mlweb.loria.fr/lalolab/lalolib.html#apiref
*/

window.onload = function() {
	// Set mouse variables
	var mousePosition,
		isMouseDown,
		isLastJointDown,
		whichArm;
	// Get Task space
	var cvsTask = document.getElementById('cvs-task');
	var ctxTask = cvsTask.getContext('2d'), 
		ctxTaskWidth = cvsTask.width,
		ctxTaskHeight = cvsTask.height,
		ctxTaskRadius = ctxTaskHeight / 2; // because height < width
	// Get Joint space
	var divJoint = document.getElementById('div-joint');
	// Get Log
	var cvsLog = document.getElementById('cvs-log'),
		ctxLog = cvsLog.getContext('2d'),
		ctxLogWidth = cvsLog.width,
		ctxLogHeight = cvsLog.height;
	// Set joint angles
	var theta1 = 80*Math.PI/180, 
		theta2 = 70*Math.PI/180, // w.r.t. theta1
		theta3 = 30*Math.PI/180, // w.r.t. theta1
		theta1Color = 'rgba(255,0,0,0.6)',
		theta2Color = 'rgba(0,255,0,0.6)',
		theta3Color = 'rgba(0,0,255,0.6)';
	// Set joint segments
	var seg1 = 70,
		seg2 = 50,
		seg3 = 30;
	// Set arrays
	var ucm_vec,
		cm1_vec,
		cm2_vec,
		ucm_vec_ext,
		ucm_vec_ctr,
		scaler = 5;
	// Plotly parameters
	var rangeScaler = 2,
		trace1,
		trace2,
		data,
		layout;

	// Add listeners
	document.addEventListener('mousemove', move, false);
	document.addEventListener('mousedown', setDraggable, false);
	document.addEventListener('mouseup', setDraggable, false);

	// Define arms
	let IK = new InverseKinematics(ctxTaskWidth/2, ctxTaskHeight/2);
	IK.addArm(seg1, theta1, theta1Color);  // bottom
    IK.addArm(seg2, theta2, theta2Color);  // center
    IK.addArm(seg3, theta3, theta3Color);  // top

    // Initial update for the synchronous execution consequently
    IK.reach(IK.xEnd, IK.yEnd, 2);
    Promise.resolve(initPlotly())
    	.then(update())
    	.then(updatePlot());
    // Promise.resolve(update()).then(initPlotly());
    writeLog(IK.angles[0], IK.angles[1], IK.angles[2]);
    writeInitMsg("Click & Drag the joints!");

	function update() {
		// Draw the circle and the arms
		ctxTask.clearRect(0, 0, ctxTaskWidth, ctxTaskHeight);
		// Add a circle to task space
		ctxTask.beginPath();
		ctxTask.arc(ctxTaskWidth/2, ctxTaskHeight/2, ctxTaskRadius, 0, 2*Math.PI);
		ctxTask.stroke();
		// Draw arms
		IK.render(ctxTask);
		ctxLog.clearRect(0,0,ctxLogWidth, ctxLogHeight);

		var jacob = getJacobian(seg1,seg2,seg3,IK.angles[0],IK.angles[1],IK.angles[2]);
		var SVD = getBases(mySVD(jacob));
		ucm_vec = mul(array2vec(SVD[0]), scaler),
		cm1_vec = array2vec(SVD[1]),
		cm2_vec = array2vec(SVD[2]);

		// Center UCM bases at the terminal end point & Extend by scaler
		ucm_vec_ext = add(entrywisediv(ucm_vec, 2), IK.angles);
		ucm_vec_ctr = sub([0,0,0], entrywisediv(ucm_vec, 2));
		ucm_vec_ctr = add(ucm_vec_ctr, IK.angles);
	}

	function updatePlot() {
		// Draw joint plot
		Plotly.restyle(divJoint, {	
			x: [[IK.angles[0]]], // double bracket!
			y: [[IK.angles[1]]],
			z: [[IK.angles[2]]], 
		}, 0); // trace1
		// Draw UCM bases
		Plotly.restyle(divJoint,{
			x: [[ucm_vec_ctr[0], ucm_vec_ext[0]]], // double bracket!
			y: [[ucm_vec_ctr[1], ucm_vec_ext[1]]],
			z: [[ucm_vec_ctr[2], ucm_vec_ext[2]]],
		}, 1) // trace2
	}

	var focused = {
		state: false,
	}

	function move(e) {
		// Mouse movement callback
		if (!isMouseDown) {
			return;
		}
		getMousePosition(e);
		// if any joint is focused
		if (focused.state) {
			IK.reach(mousePosition.x, mousePosition.y, whichArm);
			update();
			updatePlot();
			writeLog(IK.angles[0], IK.angles[1], IK.angles[2]);
			return;
		}

		// Check if the cursor hovers over a joint
		for (var i=0; i<IK.arms.length; i++) {
			let circle = IK.circles[i];
			if (cursorOverJoint(circle)) {
				whichArm = i;
				IK.reach(mousePosition.x, mousePosition.y, whichArm);
				focused.state = true;
				break;
			}
		}
		update();
	}

	function setDraggable(e) {
		var t = e.type;
		if (t === 'mousedown') {
			isMouseDown = true;
		} else if (t === 'mouseup') {
			isMouseDown = false;
			releaseFocus();
		}
	}

	function releaseFocus() {
		focused.state = false;
	}

	function getMousePosition(e) {
		let rect = cvsTask.getBoundingClientRect();
		mousePosition = {
			x: Math.round(e.x - rect.left),
			y: Math.round(e.y - rect.top)
		}
	}

	function cursorOverJoint(circle) {
		// Check if the mouse hovers over the joint angle
		// by calculating the relative distance from the circle center
		let x = mousePosition.x - circle.x;
		let y = mousePosition.y - circle.y;
		// return true if x^2 + y^2 <= radius^2
		return x*x + y*y <= circle.r*circle.r;
	}

	function cutCircle(context, x, y, radius){
	    context.globalCompositeOperation = 'destination-out'
	    context.arc(x, y, radius, 0, Math.PI*2, true);
	    context.fill();
	}

	function writeLog(theta1, theta2, theta3) {
		// Convert radians to angles
		let fixedPoint = 1;
		ang1 = (theta1*180/Math.PI).toFixed(fixedPoint);
		ang2 = (theta2*180/Math.PI).toFixed(fixedPoint);
		ang3 = (theta3*180/Math.PI).toFixed(fixedPoint);
		// Write log
		ctxLog.font = "30px Comic Sans MS";
		ctxLog.textAlign = "center";
		ctxLog.fillText("("+ang1.toString()+", "+ang2.toString()+", "+ang3.toString()+")",
			ctxLogWidth/2, ctxLogHeight/2);
	}

	function writeInitMsg(txt) {
		// Write an initial message on task space
		ctxTask.font = "20px Comic Sans MS";
		ctxTask.textAlign = "center";
		ctxTask.fillStyle = "black";
		ctxTask.fillText(txt, ctxTaskWidth/2, ctxTaskHeight*0.3);
	}

	//--------------------------------------------------------------------
	function initPlotly() {
		// Initiate plotly
		// console.log(IK.angles[0]*180/Math.PI, IK.angles[1]*180/Math.PI, IK.angles[2]*180/Math.PI,);
		var trace1 = {
			x: [IK.angles[0]],
			y: [IK.angles[1]],
			z: [IK.angles[2]],
			mode: 'markers',
			marker: {
				size: 12,
				line: {
					color: 'rgba(217, 217, 217, 0.14)',
					width: 0.5
				},
				opacity: 0
			},
			type: 'scatter3d',
		}
		var trace2 = {
			x: [],
			y: [],
			z: [],
			mode: 'lines',
	        	line: {
	        		opacity: 0.8,
	        		color: 'rgb(0,0,200)',
	        		width: 7,
	        	},
			type: 'scatter3d',
		}

		var data = [trace1, trace2];
		var layout = {
				width: 404,
				height: 354,
				margin: {
					l: 0,
					r: 0,
					b: 0,
					t: 0
		  		},
		  		showlegend: false,
		  		scene: {
		  			aspectmode: "manual", // <- critical
		  			aspectratio: {
				    	x: 1, y: 1, z: 1, // <- critical
				    },
					xaxis: {
						title: 'theta1',
						backgroundcolor: theta1Color,
						showbackground: true,
						gridcolor: "rgb(255, 255, 255)",
						zerolinecolor: "rgb(255, 255, 255)",
						range: [-Math.PI*rangeScaler, Math.PI*rangeScaler],
						nticks: 3,
					},
					yaxis: {
						title: 'theta2',
						backgroundcolor: theta2Color,
						showbackground: true,
						gridcolor: "rgb(255, 255, 255)",
						zerolinecolor: "rgb(255, 255, 255)",
						range: [-Math.PI*rangeScaler, Math.PI*rangeScaler],
						nticks: 3,
					},
					zaxis: {
						title: 'theta3',
						backgroundcolor: theta3Color,
						showbackground: true,
						gridcolor: "rgb(255, 255, 255)",
						zerolinecolor: "rgb(255, 255, 255)",
						range: [-Math.PI*rangeScaler, Math.PI*rangeScaler],
						nticks: 3,
					},
					camera: {
						eye: {x:1.5, y:1.5, z:1.5},
					},
					autorange: false,
					autosize: false,
				},
			};
		Plotly.newPlot('div-joint', data, layout);
	}

}
