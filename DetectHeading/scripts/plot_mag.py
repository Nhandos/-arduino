#!/usr/bin/env python3

import argparse
import csv
from matplotlib import pyplot as plt

def main(argv):
    
    mag_x, mag_y, mag_z = [], [], []

    # Read data
    with open(argv.datafile, "r") as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            mag_x.append(float(row['mag_x (uT)']))
            mag_y.append(float(row['mag_y (uT)']))
            mag_z.append(float(row['mag_z (uT)']))

    # 3D scatterplot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(mag_x, mag_y, mag_z, marker='o')
    ax.set_xlabel('mag_x (uT)')
    ax.set_ylabel('mag_y (uT)')
    ax.set_zlabel('mag_z (uT)')

    plt.show()


if __name__== '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('datafile', type=str, help='CSV file')
    main(parser.parse_args())

