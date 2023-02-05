
def convert(path, dest):
    f = open(path, "r")
    lines = f.readlines()    
    f.close()

    f = open(dest, "w")
    for line in lines:
        f.write(line[:2] + " " + line[3:5] + " ")
    f.close()


convert("230205_logic02.csv", "230205_logic02_2.csv")
