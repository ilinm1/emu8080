#small parser to generate instruction info from http://dunfield.classiccmp.org/r/8080.txt
#p.s: several instructions had wrong sizes

def createMasks(opcode):
    mask = ""
    ambMask = ""

    for c in opcode:
        if c != '0' and c != '1':
            mask += '0'
            ambMask += '1'
        else:
            mask += c
            ambMask += '0'
    
    return (mask, ambMask)

def processLine(line):
    toks = []
    buf = ""
    for c in line:
        if c == ' ':
            if buf != "":
                toks.append(buf)
                buf = ""
        else:
            buf += c

    mnemonic = toks[0].lower()

    opcode = ""
    for tok in toks:
        if tok.find('0') != -1 or tok.find('1') != -1:
            opcode = tok
            break

    mask, ambMask = createMasks(opcode)
    size = 1
    for tok in toks:
        if tok in ("db", "lb", "hb", "pa"):
            size += 1

    lookup = f"    0b{mask}, 0b{ambMask}, {size}, //{mnemonic}\n"
    pointers = f"    &i{mnemonic},\n"
    methods = f"//{line[:len(line) - 1]}\nvoid i{mnemonic}(Cpu* cpu, byte* mem, byte* instruction)\n{{\n}}\n\n"
    return (lookup, pointers, methods)

def process(path):
    lookup = "{\n"
    pointers = "{\n"
    methods = ""

    inf = open(path, "r")
    for l in inf.readlines():
        l, p, m = processLine(l)
        lookup += l
        pointers += p
        methods += m
    inf.close()

    lookup += "}\n\n"
    pointers += "}\n\n"

    outf = open("output.txt", "w")
    outf.write(lookup)
    outf.write(pointers)
    outf.write(methods)
    outf.close()

print("Input file path: ")
process(input())
print("Saved to output.txt")
