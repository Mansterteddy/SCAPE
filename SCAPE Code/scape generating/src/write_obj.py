import csv

def write_obj(final_point, tri):
    #f = open("D:/Data Python/SCAPE/res/res1.obj", 'w')
    f = open("../res/res1.obj", 'w')
    f.write('v 0 0 0\n')
    count = 0
    for row in final_point:
        if(count == 0):
            str_res = 'v ' + str(row)
            f.write(str_res)
            count += 1
        elif(count == 1):
            str_res = ' ' + str(row)
            f.write(str_res)
            count += 1
        else:
            str_res = ' '+ str(row) + '\n'
            f.write(str_res)
            count = 0

    for row in tri:
        str_res = 'f ' + str(row[0]) + ' ' + str(row[1]) + ' ' + str(row[2]) + '\n'
        f.write(str_res)

    f.close()


def write_obj_res():
    csv_reader = csv.reader(open("../res/res.txt"))
    f2 = open("../res/res1.obj", 'w')
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

    csv_reader = csv.reader(open("../../scape training/bodyseg/partidx/tri.txt"))
    for row in csv_reader:
        str = row[0] + '\n'
        f2.write(str)

    f2.close()

if __name__ == '__main__':
    write_obj_res()