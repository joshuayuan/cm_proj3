import math


def read_acc_from_file():
    """
        Returns a list of [ax, ay, az] read in from txt file
    """
    fname = raw_input("Read from this file:\t")
    f = open(fname, "r")
    line = f.readline()
    lines = line.split(',')

    accs = []
    for i in range(0, len(lines) - 3, 3):
        one_set_of_accs = [float(lines[i].strip()), float(lines[i+1].strip()), float(lines[i+2].strip())]
        accs.append(one_set_of_accs)
    return accs



def get_offsets(accs):
    """
        Returns phi and theta offsets as [phi, theta]
    """
    phi_offset = 0.0
    theta_offset = 0.0
    N = 100 # 'random' sample size

    ni = 1
    for ax, ay, az in accs:
        if ni > N:
            break
        phi = math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0))
        theta = math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))

        phi_offset += phi
        theta_offset += theta
        ni += 1


    return [float(phi_offset) / float(N), float(theta_offset) / float(N)]




while True:
    accs = read_acc_from_file()
    offsets = get_offsets(accs)
    print("Accelerometer offsets: " + str(offsets[0]) +", " + str(offsets[1]))


