

def convert(path, dest):
    f = open(path, "r")
    lines = f.readlines()
    f.close()

    f = open(dest, "w")
    last = ""
    cnt = 0
    rdActive = 0
    wrActive = 0
    #for i in range(len(lines[0])):
    i = 0
    while i < len(lines[0]):
        if wrActive == 0 and rdActive == 0:
            if lines[0][i:i+5] != last:
                if cnt > 1:
                    f.write(last + "  " + str(cnt) + "x\n")
                else:
                    if last[:2] == "F0":
                        f.write(last + "  CMT2300A_CUS_RSSI_DBM\n")
                    elif last[:2] == "63":
                        f.write(last + "  CMT2300A_CUS_FREQ_CHNL\n")
                    elif last[:2] == "65":
                        f.write(last + "  CMT2300A_CUS_IO_SEL\n")
                    elif last[:2] == "66":
                        f.write(last + "  CMT2300A_CUS_INT1_CTL\n")
                    elif last[:2] == "E6":
                        f.write(last + "  CMT2300A_CUS_INT1_CTL\n")
                    elif last[:2] == "67":
                        f.write(last + "  CMT2300A_CUS_INT2_CTL\n")
                    elif last[:2] == "E7":
                        f.write(last + "  CMT2300A_CUS_INT2_CTL\n")
                    elif last[:2] == "68":
                        f.write(last + "  CMT2300A_CUS_INT_EN\n")
                    elif last[:2] == "69":
                        f.write(last + "  CMT2300A_CUS_FIFO_CTL\n")
                    elif last[:2] == "E9":
                        f.write(last + "  CMT2300A_CUS_FIFO_CTL\n")
                    elif last[:2] == "6C":
                        f.write(last + "  CMT2300A_CUS_FIFO_CLR\n")
                    elif last[:2] == "46":
                        f.write(last + "  PAYLOAD_LENGTH\n")
                    elif last == "ED 10":
                        f.write(last + "  CMT2300A_CUS_INT_FLAG, PREAM_OK\n")
                    elif last == "ED 18":
                        f.write(last + "  CMT2300A_CUS_INT_FLAG, PREAM_OK, SYNC_OK\n")
                    elif last == "ED 1B":
                        f.write(last + "  CMT2300A_CUS_INT_FLAG, PREAM_OK, SYNC_OK, NODE_OK, CRC_OK\n")
                    elif last[:2] == "ED":
                        f.write(last + "  CMT2300A_CUS_INT_FLAG\n")
                    elif last == "16 04":
                        f.write(last + "  RSSI_AVG_MODE[2:0] = 0x04\n")
                    elif last == "16 0C":
                        f.write(last + "  RSSI_DET_SEL[1:0] = 0x01, RSSI_AVG_MODE[2:0] = 0x04\n")
                    elif last == "60 02":
                        f.write(last + "  CMT2300A_GO_STBY\n")
                    elif last == "60 08":
                        f.write(last + "  CMT2300A_GO_RX\n")
                    elif last == "60 10":
                        f.write(last + "  CMT2300A_GO_SLEEP\n")
                    elif last == "60 40":
                        f.write(last + "  CMT2300A_GO_TX\n")
                    elif last == "E6 0A":
                        f.write(last + "  CMT2300A_CUS_INT1_CTL -> TX_DONE\n")
                    elif last == "EA 00":
                        f.write(last + "  CMT2300A_CUS_INT_CLR1\n")
                    elif last == "6A 00":
                        f.write(last + "  CMT2300A_CUS_INT_CLR1\n")
                    elif last == "6B 00":
                        f.write(last + "  CMT2300A_CUS_INT_CLR2\n")
                    elif last == "E1 51":
                        f.write(last + "  CMT2300A_STA_SLEEP\n")
                    elif last == "E1 52":
                        f.write(last + "  CMT2300A_STA_STBY\n")
                        f.write("-------------------------------------------------------------------------------\n")
                    elif last == "E1 55":
                        f.write(last + "  CMT2300A_STA_RX\n")
                    else:
                        f.write(last + "\n")
                last = lines[0][i:i+5]
                if last[:2] == "C5":
                    if lines[0][i:i+14] == "C5 01 45 01 46":
                        wrActive = i+17
                        f.write(lines[0][i:i+5] + "\n")
                        i += 6
                        f.write(lines[0][i:i+5] + "\n")
                        i += 6
                        f.write(lines[0][i:i+5] + "  PAYLOAD_LENGTH = " + lines[0][i+3:i+5] + "\n")
                        f.write("\n###############################################################################\n")
                        f.write("TX\n")
                elif last[:5] == "ED 1B":
                    if lines[0][i:i+17] == "ED 1B 60 02 E1 52":
                        rdActive = i+17
                        f.write(lines[0][i:i+5] + "  CMT2300A_CUS_INT_FLAG, PREAM_OK, SYNC_OK, NODE_OK, CRC_OK\n")
                        i += 6
                        f.write(lines[0][i:i+5] + "  CMT2300A_GO_STBY\n")
                        i += 6
                        f.write(lines[0][i:i+5] + "  CMT2300A_STA_STBY\n")
                        f.write("\n###############################################################################\n")
                        f.write("RX\n")
                cnt = 1
            else:
                cnt += 1

            i += 6
        elif wrActive != 0:
            if (lines[0][i:i+2] == "63") and (lines[0][i+6:i+11] == "60 40"):
                wrActive = 0
                f.write("\n###############################################################################\n")
                last = ""
            else:
                f.write(lines[0][i:i+3])
                i += 3
        elif rdActive != 0:
            if (lines[0][i:i+2] == "F0") and (lines[0][i+6:i+11] == "60 10"):
                rdActive = 0
                f.write("\n###############################################################################\n")
                last = ""
            else:
                f.write(lines[0][i:i+3])
                i += 3
    f.close()

convert("230205_cap10.csv", "230205_cap10.txt")
convert("230205_cap11.csv", "230205_cap11.txt")
convert("230205_cap12.csv", "230205_cap12.txt")
convert("230205_cap13.csv", "230205_cap13.txt")
#convert("230205_cap14.csv", "230205_cap14.txt")
#convert("230205_cap15.csv", "230205_cap15.txt")
#convert("230205_cap16.csv", "230205_cap16.txt")
#convert("230205_cap17.csv", "230205_cap17.txt")
#convert("230205_cap18.csv", "230205_cap18.txt")
