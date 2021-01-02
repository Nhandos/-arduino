#!/usr/bin/env python3

import argparse
import csv
import re


def main(argv):

    PATTERN = r"[-+]?[0-9]*\.?[0-9]+"
    inputfile = argv.inputfile
    outputfile = argv.outputfile

    FIELD_NAMES = [
        "acc_x (mg)",
        "acc_y (mg)",
        "acc_z (mg)",
        "gyr_x (dps)",
        "gyr_y (dps)",
        "gyr_z (dps",
        "mag_x (uT)",
        "mag_y (uT)",
        "mag_z (uT)",
        "tmp (C)"
    ]

    with open(inputfile, "r") as serialfile, open(outputfile, "w") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=FIELD_NAMES)
        writer.writeheader()
        for line in serialfile:
            m = re.findall(PATTERN, line)
            if m:
                writer.writerow({k: v for k, v in zip(FIELD_NAMES, m)})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Convert serial output into csv')
    parser.add_argument('inputfile', type=str, help='input file')
    parser.add_argument('outputfile', type=str, help='outputfile')
    main(parser.parse_args())

