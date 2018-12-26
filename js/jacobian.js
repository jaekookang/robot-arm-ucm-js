/*
This script includes functions to compute the Jacobian matrix
as well as the null/range space for the given Jacobian.

2018-12-25 Jaekoo Kang

References:
- Matrix calculation: https://medium.com/@marielwerner28/matrix-methods-in-js-6331cc95daf7
- Singular Value Decomposition: http://mlweb.loria.fr/lalolab/lalolib.html 
- PCA: https://labriata.github.io/jsinscience/pca/index.html
*/

//Matrices from U,s,V = SVD(X)
var lab;
var mU = [],
	mV = [],
	s = [];
var ucm = [],
	cm1 = [],
	cm2 = [];

function getJacobian(l1,l2,l3,a1,a2,a3) {
	// Compute the Jacobian matrix
	let j11 = -(l1*Math.sin(a1)+l2*Math.sin(a1+a2)+l3*Math.sin(a1+a2+a3)),
		j12 = -(l2*Math.sin(a1+a2)+l3*Math.sin(a1+a2+a3)),
		j13 = -l3*Math.sin(a1+a2+a3),
		j21 = l1*Math.cos(a1)+l2*Math.cos(a1+a2)+l3*Math.cos(a1+a2+a3),
		j22 = l2*Math.cos(a1+a2)+l3*Math.cos(a1+a2+a3),
		j23 = l3*Math.cos(a1+a2+a3);
	let row1 = [j11, j12, j13],
		row2 = [j21, j22, j23];

	return [row1, row2];
}

function mySVD(jacobianArray) {
	// Compute the Singular Value Decomposition on jacobianArray
	//Matrices from U,s,V = SVD(X)
	mU = [],
	mV = [],
	s = [];

	var lab = new Lalolab("myLab",false,'js/lalolib/');
	let jacobianMatrix = array2mat(jacobianArray);
	// let expr = svd(jacobianMatrix);
	// Computation
	lab.load(jacobianMatrix,"X");
	lab.do("svd(X',\"full\")", function (result) {
	        //These loops build arrays containing the U and V matrices for further handling
	        //from the matrix objects contained by result
	        for (i=0;i<result.U.m;i++) {
	          var tmp=[];
	          for (j=0;j<result.U.n;j++) {
	            tmp.push(result.U.val[i*result.U.n + j])
	          }
	          mU.push(tmp)
	        }
	        for (i=0;i<result.V.m;i++) {
	          var tmp=[];
	          for (j=0;j<result.V.n;j++) {
	            tmp.push(result.V.val[i*result.V.n + j])
	          }
	          mV.push(tmp)
	        }
	        s.push(result.s);
	  	  });
	lab.close();
	// return [mU, s, mV]; 
}

function getBases() {
	// Compute the UCM and the CM bases
	// only when the dimensions are met
	if (s.length === mU.length-1) {
		// Retrieve UCM bases
		for (var i=0; i<mU.length; i++) {
			ucm[i] = mU[mU.length-1][i];
		}
		// Retrieve CM bases
		for (var i=0; i<mU.length; i++) {
			cm1[i] = mU[0][i]; // first CM base
			cm2[i] = mU[1][i]; // second CM base
		}
	}
	return [ucm, cm1, cm2];
}

