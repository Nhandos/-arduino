#!/usr/bin/env python3

import argparse
import csv
import numpy as np
from matplotlib import pyplot as plt


def cost_function(x, *args, **kwargs):
    """Ellipsoid fitting cost function for calibrating MEMs magnetomer

    For any given orientation of the magnetometer, the magnitude of the 
    magnetic vector should be a constant equal to the local geomagnetic 
    field strength.

    Mathematically, the measurements of an ideal of magnetometer will
    lie on the surface of a sphere with radius equal to B the local
    geomagnetic field strength, and origin at zero. 
    However, the effects of soft/hard irons nearby the sensor 
    will distort the measurements to lie on the surface of a ellipsoid.

    The general expression for the locus of a vector 'u' lying on the
    surface of an ellipsoid with center at u_0 is:

    (u - u_0)^T * A * (u - u_0) = const

    where A is a symmetric matrix defining the shape of the ellipsoid

        
    """

    pass


def estCalibrationParams_7(X):

    """

    Args:
        X = M by 3 matrix of uncalibrated magnetometer measurements

    """

    # Transform X to the M by 7 measurement matrix
    tmp = np.zeros((X.shape[0], 7), dtype=np.float64)
    tmp[:, 3:6] = X[:,0:3]
    tmp[:, 0:3] = X[:, 0:3]**2
    tmp[:,-1] = 1
    X = tmp 

    # Solve using eigen-decomposition approach since the model being fitted
    # is a homogeneous model
    XT_X = np.matmul(X.T, X)

    # Find eigen values of norm matrix
    eigVal, eigVec= np.linalg.eigh(XT_X, 'L')    

    # the solution vector is the vector associated iwth the smallest eigen
    # value
    beta = eigVec[np.argmin(eigVal),:]
    

    # Ellipsoid Fit matrix
    det = 1.0
    A = np.zeros((3,3),dtype=np.float64)
    for i in range(3):
        A[i,i] = beta[i]

    det = np.linalg.det(A)

    # if determinant is negative, then negate value in solution vector beta
    # to ensure that determinant positive in A
    if det < 0.0:
        beta = -beta
        for i in range(3):
            A[i,i] = beta[i]

        det = np.linalg.det(A)

    A /= (det ** (1./3))

    print('Solution Vector: \n' + str(beta))
    print('Ellipsoid fit matrix: \n' + str(A))
    
    # Hard Iron Vector
    V = np.zeros((3, 1), dtype=np.float64)
    for i in range(3):
        V[i] = -1./2 * beta[i + 3]/beta[i]
    print('Hard iron vector: \n' + str(V))

    # Inverse Soft iron matrix
    W_inv = np.sqrt(A)
    print('Inverse soft iron matrix: \n' + str(W_inv))

    # Geometric field strength
    B = abs(A[0,0]*V[0]**2 + A[1,1]*V[1]**2 + A[2,2]*V[2]**2 - beta[6])
    B = B ** (1./2)

    print('Geometric field strength: {} uT'.format(B))

    # Fit error
    err = 1./2./B/B * (min(eigVal)/X.shape[0]) ** 2
    print('Fit error: {}'.format(err))


    # reconstruct points
    biasVec = -np.matmul(W_inv, V)
    correctionMat = np.vstack((W_inv, biasVec.T))
    print(correctionMat)
    XCalib = np.zeros((X.shape[0], 4),dtype=np.float64)
    XCalib = np.matmul(X[:,3:], correctionMat)

    # 3D scatterplot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(tmp[:,3], tmp[:,4], tmp[:,5], marker='o')
    ax.scatter(XCalib[:,0], XCalib[:,1], XCalib[:,2], marker='^', color='r')
    ax.set_xlabel('mag_x (uT)')
    ax.set_ylabel('mag_y (uT)')
    ax.set_zlabel('mag_z (uT)')

    plt.show()

    
def main(argv):
    X = None
    with open(argv.datafile, "r") as csvfile:
        reader = list(csv.DictReader(csvfile))
        X = np.zeros((len(reader), 3))
        for i, row in enumerate(reader):
            mag_sample = [row['mag_x (uT)'],
                          row['mag_y (uT)'],
                          row['mag_z (uT)']]

            X[i,:] = list(map(float, mag_sample))

    estCalibrationParams_7(X)



if __name__== '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('datafile', type=str, help='CSV file')
    main(parser.parse_args())
