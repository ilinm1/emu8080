#for converting intel hex format to plain binary file

def processLine(line):
    start = line.find(':')
    if len(line[start:]) < 8:
        raise RuntimeError("Line is too short")

    #entry format:
    #comments or whatever :CCOOOOTTDDDD...SSS...
    #where C is byte count in data section from 0x00 to 0xFF
    #O is offset relatively to RAM start from 0x0000 to 0xFFFF
    #T is entry type, 0x00 for data, 0x01 for EOF, the rest is not important for 8080
    #D is data bytes
    #S is checksum

    count = int(line[start + 1 : start + 3], 16)
    offset = int(line[start + 3 : start + 7], 16)
    recordType = int(line[start + 7 : start + 9], 16)

    if recordType == 0:
        data = []
    
        for i in range(start + 9, start + 9 + count * 2, 2):
            data.append(int(line[i:i+2], 16))

        return bytes(data), offset
    elif recordType == 1:
        print("EOF")
        return None, 0
    else:
        raise RuntimeError("Unsupported record type")

def process(path):
    inf = open(path, "r")
    outf = open("output.bin", "wb+")

    for l in inf.readlines():
        data, offset = processLine(l)
        if data != None:
            outf.seek(offset)
            outf.write(data)
    inf.close()
    outf.close()

print("Input file path: ")
process(input())
print("Saved to output.bin")
