  .syntax unified @ for thumb-2
  .thumb
  .text
  .global main
  .balign 4
  .thumb_func
  .type main, %function
main:
  MOV r1,#0
loop:
  LDRB r2,[r0],#1
  CMP r2,#0
  IT NE
  ADDNE r1,r1,#1
  BNE loop
  MOV r0,r1
  BX lr

  .data
str: .asciz "Hello World"
  .end
