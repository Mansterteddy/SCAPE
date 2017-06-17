#Function that read corresponding file, return list data structure

import csv
import numpy as np

'''
Read whole.txt file.
'''
def handle_whole(file):
    csv_reader = csv.reader(open(file))
    res = []
    for row in csv_reader:
        res_son = []
        for item in row[0].split(' '):
            if(item == 'v'):
                pass
            else:
                res_son.append(float(item))
        res.append(res_son)

    return np.array(res)

'''
Read tri.txt file.
'''
def handle_tri(file):
    csv_reader = csv.reader(open(file))
    res = []
    for row in csv_reader:
        res_son = []
        for item in row[0].split(' '):
            if(item == 'f'):
                pass
            else:
                res_son.append(int(item))
        res.append(res_son)

    return res

'''
Read tripart.txt file.
'''
def handle_tripart(file):
    csv_reader = csv.reader(open(file))
    res = []
    for row in csv_reader:
        res.append(int(row[0]))

    return res

'''
Read .obj file.
'''
def handle_obj(file):
    csv_reader = csv.reader(open(file))
    res = []
    for row in csv_reader:
        if(row[0][0] == 'v'):
            res_son = []
            for item in row[0].split(' '):
                if(item == 'v'):
                    pass
                else:
                    res_son.append(float(item))
            res.append(res_son)
        else:
            pass
    return np.array(res)

'''
Read files like lefthand.txt.
'''
def handle_txt(i):
    segname = {1: "lefthand", 2: "lowerarm", 3: "uparm", 4: "righthand", 5: "rightlowarm", 6: "rightuparm",
               7: "leftfoot", 8: "leftlowleg", 9: "leftupleg",
               10: "rightfoot", 11: "rightlowleg", 12: "rightupleg", 13: "head", 14: "chest", 15: "stomach", 16: "hip"}
    #filename = "D:/matlab_code/scapecode/bodyseg/partidx/" + segname[i] + ".txt"
    filename = "../../scape training/bodyseg/partidx/" + segname[i] + ".txt"
    csv_reader = csv.reader(open(filename))
    res = []
    for row in csv_reader:
        res.append(int(row[0].split(' ')[0]) - 1)
    return res

'''
Template of read file.
'''
def handle(file):
    csv_reader = csv.reader(open(file))
    res = []
    for row in csv_reader:
        res_son = []
        for item in row[0].split(' '):
            if(item == 'f' or item == 'v'):
                pass
            else:
                res_son.append(int(item))
        res.append(res_son)
    print len(res)
