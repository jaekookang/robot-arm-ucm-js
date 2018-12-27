class InverseKinematics {
	constructor(x, y) {
		this.x = x; // base x
		this.y = y; // base y
		this.arms = [];
		this.angles = new Array(3); // angles w.r.t. each joint
		this.xEnd = null;
		this.yEnd = null;
		this.angleSum = 0;
		this.circles = [];
		this.lastArm = null;
		this.circleRadius = 8;
	}

	addArm(length, angle, color) {
		// Update arm if parents exist
		if (this.lastArm) {
			// Use the relative angle
			var arm = new Arm(0, 0, length, this.angleSum+angle);
			var xy = this.lastArm.getEndXY();
			arm.x = xy[0]; // update x origin
			arm.y = xy[1]; // update y origin
			arm.parent = this.lastArm;
			// Add child
			this.arms[this.arms.length-1].child = arm;
		} else {
			var arm = new Arm(0, 0, length, angle);
			arm.x = this.x; // update x origin
			arm.y = this.y; // update y origin
		}
		// Update
		this.arms.push(arm); // [old, ... ,new]
		this.lastArm = arm;
		this.circles.push(new drawJointCircle(0, 0, this.circleRadius, color, 'black'));
		this.xEnd = arm.getEndXY()[0];
		this.yEnd = arm.getEndXY()[1];
		this.angleSum += angle;
	}

	render(context) {
		// Draw the entire arms
		for (var i=0; i<this.arms.length; i++) {
			let arm = this.arms[i];
			let circle = this.circles[i];
			let xy = arm.getEndXY();

			arm.render(context);
			// Add a joint at the tip
			circle.x = xy[0];
			circle.y = xy[1];
			circle.draw(context);
		}
	}

	drag(x, y, whichArm) {
		// this.lastArm.drag(x, y);
		this.arms[whichArm].drag(x, y);
	}

	reach(x, y, whichArm) {
		// Drag & Update all the arms
		// whichArm begins from 0 (root), 1, 2 ...
		this.drag(x, y, whichArm);
		this.update(whichArm);
	}

	update(whichArm) {
		// to the root
		for (var i=0; i<=whichArm; i++) {
			let arm = this.arms[i];
			if (arm.parent) {
				let xy = arm.parent.getEndXY();
				arm.x = xy[0];
				arm.y = xy[1];
			} else {
				arm.x = this.x;
				arm.y = this.y;
			}

		}
		// to the branch
		for (var i=whichArm; i<this.arms.length-1; i++) {
			let arm = this.arms[i];
			let xy = arm.getEndXY();
			if (arm.child) {
				arm.child.x = xy[0];
				arm.child.y = xy[1];
			}
		}
		/*
		TODO: Update branches moving passively like the roots
		See the following commented code.
		*/

		// if (whichArm<this.arms.length-1) {
		// 	for (var i=whichArm; i<this.arms.length-1; i++) {
		// 		console.log('to the branch '+(i+1).toString());
		// 		let arm = this.arms[(i+1)];
		// 		// Reverse arm's ends
		// 		var xyCurr = arm.getEndXY();
		// 		arm.x = xyCurr[0];
		// 		arm.y = xyCurr[1];
		// 		arm.angle += Math.PI;

		// 		// Drag to the parent's end
		// 		var xyPrev = arm.parent.getEndXY()
		// 		arm.drag(xyPrev[0], xyPrev[1]);

		// 		// Reverse arm's ends again (back to original)
		// 		var xyCurr = arm.getEndXY();
		// 		arm.x = xyCurr[0];
		// 		arm.y = xyCurr[1];
		// 		arm.angle += Math.PI;
		// 		}
		// 	}

		// Update the circles
		for (var i=0; i<this.arms.length; i++) {
			let arm = this.arms[i];
			let circle = this.circles[i];
			let xy = arm.getEndXY();
			// Add a joint at the tip
			circle.x = xy[0];
			circle.y = xy[1];
		}

		// Get absolute angles
		let angles = [];
		for (var i=0; i<this.arms.length; i++) {
			angles.push(this.arms[i].angle);
		}
		// Get relative angles
		this.angles[0] = angles[0];
		for (var i=angles.length-1; i>0; i--) {
			this.angles[i] = angles[i] - angles[i-1];
		}
		// Update end points
		this.xEnd = this.lastArm.getEndXY()[0];
		this.yEnd = this.lastArm.getEndXY()[1];
	}
}

class drawJointCircle {
	/*
	This generates a circle
	*/
	constructor(x, y, r, fill, stroke) {
		this.x = x;
		this.y = y;
		this.r = r;
		this.fill = fill;
		this.stroke = stroke;
	}

	draw(context) {
		context.beginPath();
		context.arc(this.x, this.y, this.r, 0, 2*Math.PI);
		context.fillStyle = this.fill;
		context.lineWidth = 3;
		context.fill();
		context.strokeStyle = this.stroke;
		context.stroke();
	}
}