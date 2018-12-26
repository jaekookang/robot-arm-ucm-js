class Arm {
	/*
	* This class creates a single arm
 	* 
	* 2018-12-24 Jaekoo Kang
	* 
	* Note:
 	* - Angles are in radian.
	* 
	* Thanks to:
	* - Coding Math on YouTube: https://www.youtube.com/user/codingmath
	*/
	constructor(x, y, length, angle) {
		this.x = x; // origin x
		this.y = y; // origin y
		this.length = length;
		this.angle = angle;
		this.parent = null; // 'parent' means heading toward the root
		this.child = null;
	}

	getBegXY() {
		// Get xy coordinates of the first joint
		return [this.x, this.y]
	}

	getEndXY() {
		// Get xy coordinates of the last joint
		return [this.x + this.length*Math.cos(this.angle), 
				this.y + this.length*Math.sin(this.angle)]
	}

	getAngles() {
		// Get all angles from parents to the current
		return this.angles;
	}

	render(context) {
		let xy = this.getEndXY();
		// Add a circle
		// Add arm
		context.strokeStyle = "#000000";
		context.lineWidth = 2;
		context.beginPath();
		context.moveTo(this.x, this.y);
		context.lineTo(xy[0], xy[1]);
		context.stroke();
	}

	getAngleAt(x, y) {
		// Get angle at the new point
		let dx = x - this.x,
			dy = y - this.y;
		return Math.atan2(dy, dx);
	}

	drag(x, y) {
		// Drag (orient) the end point to (x, y)
		this.angle = this.getAngleAt(x, y); // Update the angle
		this.x = x - Math.cos(this.angle)*this.length; // update x origin
		this.y = y - Math.sin(this.angle)*this.length; // update y origin
		if (this.parent) {
			this.parent.drag(this.x, this.y); // Update following arms
		}
	};
}

