import csv

def write_obj_test():
    f = open("D:/Data Python/SCAPE/res/1.obj", 'w')
    f.write('v 0 0 0\n')
    f.write('v 100 100 100\n')
    f.close()
    print 'Done'

def write_obj_res():
    csv_reader = csv.reader(open("D:/Data Python/SCAPE/res/res.txt"))
    f2 = open("D:/Data Python/SCAPE/res/res1.obj", 'w')
    f2.write('v 0 0 0\n')
    count = 0
    for row in csv_reader:
        if(count == 0):
            str = 'v ' + row[0]
            f2.write(str)
            count += 1
        elif(count == 1):
            str = ' ' + row[0]
            f2.write(str)
            count += 1
        else:
            str = ' '+ row[0] + '\n'
            f2.write(str)
            count = 0

    csv_reader = csv.reader(open("D:/matlab_code/scapecode/bodyseg/partidx/tri.txt"))
    for row in csv_reader:
        str = row[0] + '\n'
        f2.write(str)

    f2.close()

def write_obj(x, tri):
    f = open("D:/Data Python/SCAPE/res/res1.obj", 'w')
    f.write('v 0 0 0\n')
    for i in xrange(len(x)):
        str = 'v ' + str(x[i][0]) + ' ' + str(x[i][1]) + ' ' + str(x[i][2]) + '\n'
        f.write(str)
    for i in xrange(len(tri)):
        str = 'f ' + str(tri[i][0]) + ' ' + str(tri[i][1]) + ' ' + str(tri[i][2]) + '\n'
        f.write(str)

    f.close()


if __name__ == '__main__':
    write_obj_res()