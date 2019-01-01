/*
 * 2018-12-23 Jaekoo Kang
 *
 * Note:
 * - To run locally, use `livereload` on Terminal.
 * - After importing lalolib, all the basic math functions are overriden 
 *   if their names overlap (eg, rand) -> see LALOLib.
 * - For sharing local server, use localtunnel (https://localtunnel.github.io/www/).
 *   e.g. lt --port 35729
 *
 * TODO:
 * - Add iPhone/iPad touch movement
 * - Make UCM space as a curve
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
		whichArm,
		mouse = {
			state: false,
		};
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
	var theta1 = -80*Math.PI/180, 
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
		ucm_vec_end,
		ucm_vec_beg,
		scaler = 2,
		IK,
		xEndPrev = [],
		yEndPrev = [];
	// Plotly parameters
	var rangeScaler = 2,
		trace1,
		trace2,
		data,
		layout;
	// Slider parameter
	var slider = {
		state: false,
	}
	// Touch movement
	var touch;

	// Add listeners
	document.addEventListener('mousemove', move, false);
	document.addEventListener('mousedown', setDraggable, false);
	document.addEventListener('mouseup', setDraggable, false);
	// // track touch movement
 //    cvsTask.addEventListener('touchstart', function(e) {touchDown(e);}, false);
	// cvsTask.addEventListener('touchmove', function(e) {touchMove(e);}, false);

	// function touchMove(e) {
	//     e.preventDefault();
	//     touch = e.touches[0];

	//     move(touch);

	//     document.getElementById("xtext").innerHTML = touch.pageX;
	//     document.getElementById("ytext").innerHTML = touch.pageY;   
	// }

	// function touchUp(e) {
	// 	setDraggable(e);
	// }

	// function touchDown(e) {
	// 	setDraggable(e);
	// }
	

	// Initialization
	if (!slider.state) {
		// Define arms
		var IK = new InverseKinematics(ctxTaskWidth/2, ctxTaskHeight/2);
		IK.addArm(seg1, theta1, theta1Color);  // bottom
	    IK.addArm(seg2, theta2, theta2Color);  // center
	    IK.addArm(seg3, theta3, theta3Color);  // top
	    // Set task variables
		xEndPrev.push(IK.xEnd),
		yEndPrev.push(IK.yEnd);
		// Update
	    IK.reach(IK.xEnd, IK.yEnd, 2);
	    initPlotly();
	    update();
	    updateUCM();
	    updatePlot(IK.angles[0], IK.angles[1], IK.angles[2]);
	    writeInitMsg("Click & Drag the joints!");
	}

    $("#slider-ucm").on("input", function() {
    	/* Update Joint and Task space */
    	let dx = (ucm_vec_end[0]-ucm_vec_beg[0])*this.value;
    	let dy = (ucm_vec_end[1]-ucm_vec_beg[1])*this.value;
    	let dz = (ucm_vec_end[2]-ucm_vec_beg[2])*this.value;
    	// Update Task space
    	IK = new InverseKinematics(ctxTaskWidth/2, ctxTaskHeight/2);
    	IK.addArm(seg1, ucm_vec_beg[0]+dx, theta1Color);  // bottom
	    IK.addArm(seg2, ucm_vec_beg[1]+dy, theta2Color);  // center
	    IK.addArm(seg3, ucm_vec_beg[2]+dz, theta3Color);  // top
	    IK.reach(IK.xEnd, IK.yEnd, 2);
	    update(); // Update Task space
	    updatePlot(ucm_vec_beg[0]+dx, ucm_vec_beg[1]+dy, ucm_vec_beg[2]+dz); // Update Joint space

	    slider.state = true;
    });

	function update() {
		/* Draw the circle and the arms */
		ctxTask.clearRect(0, 0, ctxTaskWidth, ctxTaskHeight);
		// Add a circle to task space
		ctxTask.beginPath();
		ctxTask.arc(ctxTaskWidth/2, ctxTaskHeight/2, ctxTaskRadius, 0, 2*Math.PI);
		ctxTask.stroke();
		// Draw arms
		IK.render(ctxTask);
		ctxLog.clearRect(0,0,ctxLogWidth, ctxLogHeight);
		// console.log("End:", IK.xEnd, IK.yEnd);
		writeLog(IK.angles[0], IK.angles[1], IK.angles[2], IK.xEnd, IK.yEnd);
	}

	function updateUCM() {
		var jacob = getJacobian(seg1,seg2,seg3,IK.angles[0],IK.angles[1],IK.angles[2]);
		bases = getBases(jacob);
		ucm_vec = mul(bases[0], scaler);
		cm1_vec = mul(bases[1][0], scaler);
		cm2_vec = mul(bases[1][1], scaler);

		// Center UCM bases at the terminal end point & Extend by scaler
		// ucm_vec_end: the end of the ucm vector
		// ucm_vec_beg: the other end of the ucm vector
		ucm_vec_end = add(entrywisediv(ucm_vec, 2), IK.angles);
		ucm_vec_beg = sub([0,0,0], entrywisediv(ucm_vec, 2));
		ucm_vec_beg = add(ucm_vec_beg, IK.angles);
	}

	function updatePlot(ang1, ang2, ang3) {
		// Draw joint plot
		Plotly.restyle(divJoint, {	
			x: [[ang1]], // double bracket!
			y: [[ang2]],
			z: [[ang3]], 
		}, 0); // trace1
		// Draw UCM bases
		Plotly.restyle(divJoint,{
			x: [[ucm_vec_beg[0], ucm_vec_end[0]]], // double bracket!
			y: [[ucm_vec_beg[1], ucm_vec_end[1]]],
			z: [[ucm_vec_beg[2], ucm_vec_end[2]]],
		}, 1) // trace2
	}

	//----- Mouse -------------------------------------------------------------
	function move(e) {
		// Mouse movement callback
		if (!isMouseDown) {
			return;
		}
		getMousePosition(e);
		// if any joint is focused
		if (mouse.state) {
			IK.reach(mousePosition.x, mousePosition.y, whichArm);
			update(); // Update Task space
			updateUCM(); // Update UCM
			updatePlot(IK.angles[0], IK.angles[1], IK.angles[2]); // Update Joint space
			
			// Re-center the slider
			document.getElementById("slider-ucm").value = 0.5;
			xEndPrev.push(IK.xEnd),
			yEndPrev.push(IK.yEnd);
			return;
		}

		// Check if the cursor hovers over a joint
		for (var i=0; i<IK.arms.length; i++) {
			let circle = IK.circles[i];
			if (cursorOverJoint(circle)) {
				whichArm = i;
				IK.reach(mousePosition.x, mousePosition.y, whichArm);
				mouse.state = true;
				slider.state = false;
				update();
				updateUCM();
				break;
			}
		}
	}

	function setDraggable(e) {
		var t = e.type;
		if (t === 'mousedown') {
			isMouseDown = true;
		} else if (t === 'mouseup') {
			isMouseDown = false;
			// Block mouse update by setting it false
			releaseFocus();
		}
	}

	function releaseFocus() {
		mouse.state = false;
	}

	function getMousePosition(e) {
		let rect = cvsTask.getBoundingClientRect(),
			posX = e.x,
			posY = e.y;
		mousePosition = {
			x: Math.round(posX - rect.left),
			y: Math.round(posY - rect.top)
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

	//----- Log -----------------------------------------------------------
	function writeLog(theta1, theta2, theta3, xEnd, yEnd) {
		// Convert radians to angles
		let fixedPoint = 0,
			ang1 = (theta1*180/Math.PI).toFixed(fixedPoint),
			ang2 = (theta2*180/Math.PI).toFixed(fixedPoint),
			ang3 = (theta3*180/Math.PI).toFixed(fixedPoint),
			error;
		xEnd = xEnd.toFixed(0);
		yEnd = yEnd.toFixed(0);
		if (xEndPrev.length > 3 & yEndPrev.length > 3) {
			// To clear the memory
			delete xEndPrev[0];
			delete yEndPrev[0];
		}

		// Write Joint angles (thetas)
		ctxLog.font = "20px Comic Sans MS";
		ctxLog.textAlign = "center";
		ctxLog.fillText("Joints (angles): ("+ang1.toString()+", "+ang2.toString()+", "+ang3.toString()+")",
			ctxLogWidth/2, ctxLogHeight/2*0.8);
		// Write Task xy-coordinates and error (amount of deviation at the tip)
		let dx = Math.pow(xEnd - xEndPrev[xEndPrev.length-1], 2),
			dy = Math.pow(yEnd - yEndPrev[yEndPrev.length-1], 2);
		if (slider.state) {
			error = Math.sqrt(dx+dy).toFixed(1);
		} else {
			error = 0;
		}
		
		ctxLog.font = "20px Comic Sans MS";
		ctxLog.textAlign = "center";
		ctxLog.fillText("Task (x,y): ("+xEnd.toString()+", "+yEnd.toString()+") (Error: "+error.toString()+" px)",
			ctxLogWidth/2, ctxLogHeight/2*1.2);
	}

	function writeInitMsg(txt) {
		// Write an initial message on task space
		ctxTask.font = "20px Comic Sans MS";
		ctxTask.textAlign = "center";
		ctxTask.fillStyle = "black";
		ctxTask.fillText(txt, ctxTaskWidth/2, ctxTaskHeight*0.7);
	}

	//----- Plot ------------------------------------------------------------
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
					width: 5
				},
				color: 'rgb(0,191,255)',
				opacity: 0.85
			},
			type: 'scatter3d',
			name: 'Point',
		}
		var trace2 = {
			x: [],
			y: [],
			z: [],
			mode: 'lines',
	        	line: {
	        		opacity: 0.8,
	        		color: 'rgb(0,0,0)',
	        		width: 7,
	        	},
			type: 'scatter3d',
			name: 'Basis',
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
						title: 'angle1',
						backgroundcolor: theta1Color,
						showbackground: true,
						gridcolor: "rgb(255, 255, 255)",
						zerolinecolor: "rgb(255, 255, 255)",
						range: [-Math.PI*rangeScaler, Math.PI*rangeScaler],
						nticks: 3,
						autorange: false,
						autosize: false,
					},
					yaxis: {
						title: 'angle2',
						backgroundcolor: theta2Color,
						showbackground: true,
						gridcolor: "rgb(255, 255, 255)",
						zerolinecolor: "rgb(255, 255, 255)",
						range: [-Math.PI*rangeScaler, Math.PI*rangeScaler],
						nticks: 3,
						autorange: false,
						autosize: false,
					},
					zaxis: {
						title: 'angle3',
						backgroundcolor: theta3Color,
						showbackground: true,
						gridcolor: "rgb(255, 255, 255)",
						zerolinecolor: "rgb(255, 255, 255)",
						range: [-Math.PI*rangeScaler, Math.PI*rangeScaler],
						nticks: 3,
						autorange: false,
						autosize: false,
					},
					camera: {
						eye: {x:1.5, y:1.5, z:1.5},
					},
					
				},
			};
		Plotly.newPlot('div-joint', data, layout);
	}

}
