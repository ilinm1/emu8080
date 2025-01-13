conport equ 1

jmp begin

text: 
db "Hello world!$"
ds 15

;Print out a string
;Inputs:
;B:C - string's address
print:
ldax B
cpi 0x24 ;$
rz
out conport
inx B
jmp print

begin:
lxi B,text
call print
