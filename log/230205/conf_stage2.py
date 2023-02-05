

def convert(path, dest):
    f = open(path, "r")
    lines = f.readlines()
    f.close()

    readCmd = ["E6 0A", "ED 00", "EA 00", "6A 00", "6B 00", "E9 02", "69 02", "6C 02", "16 0C", "63 01", "60 08", "E1 59", "E1 55", "ED 00", "60 02", "E1 52"]

    rdOk = 0
    stack = []
    cnt = 0
    f = open(dest, "w")
    for line in lines:
        if(line[:4] == "----"):
           rdOk = 0
           continue
        if (line[:5] == readCmd[rdOk]) or (line[:2] == "63" and 9 == rdOk):
            stack.append(line)
            if rdOk == 15:
                stack.clear()
                cnt += 1
            else:
                rdOk += 1
        else:
            if cnt > 0:
                f.write("read cmd " + str(cnt)+ "x\n")
                cnt = 0
            if len(stack) > 0:
                for st in stack:
                    f.write(st)
                stack.clear()
            f.write(line)
            rdOk = 0

    f.close()

convert("230205_logic01.txt", "230205_logic01_stage2.txt")
convert("230205_logic02.txt", "230205_logic02_stage2.txt")
