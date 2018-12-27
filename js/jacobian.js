/*
This script includes functions to compute the Jacobian matrix
as well as the null/range space for the given Jacobian.

2018-12-25 Jaekoo Kang

References:
- Matrix calculation: https://medium.com/@marielwerner28/matrix-methods-in-js-6331cc95daf7
- Singular Value Decomposition: http://mlweb.loria.fr/lalolab/lalolib.html 
- PCA: https://labriata.github.io/jsinscience/pca/index.html
*/

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
	// Matrices from U,s,V = SVD(X)
	let mU = [],
	    mV = [],
	    s = [];

	let jacobianMatrix = array2mat(jacobianArray);
	// let SVD = svd(transpose(jacobianMatrix));
	let SVD = svd(transpose(jacobianMatrix), "full");

    // These loops build arrays containing the U and V matrices for further handling
    // from the matrix objects contained by result
    for (i=0;i<SVD.U.m;i++) {
      var tmp=[];
      for (j=0;j<SVD.U.n;j++) {
        tmp.push(SVD.U.val[i*SVD.U.n + j])
      }
      mU.push(tmp)
    }
    for (i=0;i<SVD.V.m;i++) {
      var tmp=[];
      for (j=0;j<SVD.V.n;j++) {
        tmp.push(SVD.V.val[i*SVD.V.n + j])
      }
      mV.push(tmp)
    }
    s.push(SVD.s);

	return [mU, s, mV]; 
}

function getBases(svdResult) {
	// Compute the UCM and the CM bases
	let mU = svdResult[0],
		s = svdResult[1],
		mV = svdResult[2];
	let ucm = [],
		cm1 = [],
		cm2 = [];

	// only when the dimensions are met
	// Retrieve UCM bases
	for (var i=0; i<mU.length; i++) {
		ucm[i] = mU[mU.length-1][i];
	}
	// Retrieve CM bases
	for (var i=0; i<mU.length; i++) {
		cm1[i] = mU[0][i]; // first CM base
		cm2[i] = mU[1][i]; // second CM base
	}
	return [ucm, cm1, cm2];
}

