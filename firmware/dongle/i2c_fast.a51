; Fast I2C bit-bang primitives for FX2LP.
; SDA = IOA.0 (bit 0x80), SCL = IOA.1 (bit 0x81)
; OEA = SFR 0xB2: 0xFE = SDA input, 0xFF = SDA output
; SCL is push-pull with clock-stretch support: after driving
; SCL high, poll P0.1 until the slave releases it.
;
; Delay is a JIT NOP sled: .ds fills with 0x00 (NOP on 8051).
; i2c_set_delay() writes RET (0x22) at the desired offset to
; control how many NOPs execute before returning.
;
; SDCC calling convention: first BYTE arg in dpl, return in dpl.

    .area DSEG (DATA)
_delay_count:
    .ds 1                ; current RET offset — must be in RAM, not CODE

    .area CSEG (CODE)

    .globl _i2c_set_delay
    .globl _i2c_write_byte
    .globl _i2c_read_byte
    .globl _i2c_start
    .globl _i2c_stop
    .globl _i2c_repeated_start
    .globl _i2c_write_verify

; ============================================================
; void i2c_set_delay(WORD ticks)
; SDCC passes first WORD arg in dph:dpl. We use low byte only.
; Clamp to [5, 126], move the RET byte in the NOP sled.
; ============================================================
_i2c_set_delay:
    mov  a, dpl          ; new count (low byte of ticks)
    mov  r6, a           ; save in r6

    ; clamp min
    clr  c
    subb a, #5
    jnc  _sd_minok
    mov  r6, #5
_sd_minok:
    mov  a, r6
    clr  c
    subb a, #127
    jc   _sd_maxok
    mov  r6, #126
_sd_maxok:

    ; clear old RET: sled[delay_count] = NOP
    mov  a, _delay_count
    add  a, #<_i2c_delay_sled
    mov  dpl, a
    clr  a
    addc a, #>_i2c_delay_sled
    mov  dph, a
    clr  a
    movx @dptr, a

    ; place new RET: sled[r6] = 0x22
    mov  a, r6
    add  a, #<_i2c_delay_sled
    mov  dpl, a
    clr  a
    addc a, #>_i2c_delay_sled
    mov  dph, a
    mov  a, #0x22
    movx @dptr, a

    ; update stored count
    mov  _delay_count, r6
    ret

; ============================================================
; NOP sled for JIT delay (127 NOPs + RET)
; i2c_set_delay() overwrites byte [N] with RET to set delay.
; Must use .db 0 (not .ds) so bytes appear in the hex file.
; ============================================================
_i2c_delay_sled:
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  ; 16
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  ; 32
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  ; 48
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  ; 64
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  ; 80
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  ; 96
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  ; 112
    .db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0    ; 127
    .db 0x22                             ; RET

; ============================================================
; Drive SCL high and wait for slave to release (clock stretch).
; Releases the line (SCL = input) then polls until SCL reads
; high, matching the vendor dongle's approach. Restores SCL to
; push-pull output before returning.
; Timeout after ~65536 iterations to avoid hanging forever.
; ============================================================
_scl_high:
    setb 0x81            ; latch SCL high
    anl  0xB2, #0xFD     ; OEA &= ~0x02: SCL = input (release line)
    mov  r3, #0          ; outer counter (256)
_sh_outer:
    mov  r2, #0          ; inner counter (256)
_sh_inner:
    jb   0x81, _sh_done  ; pin high? slave released
    djnz r2, _sh_inner
    djnz r3, _sh_outer
_sh_done:
    orl  0xB2, #0x02     ; OEA |= 0x02: SCL = output (drive high)
    ret

; ============================================================
; BOOL i2c_write_verify(void)
; Post-write clock-stretch check. Toggle SCL up to 200 times;
; if slave ever holds SCL low, write was accepted (return 1).
; If 200 tries pass without stretch, write was rejected (return 0).
; ============================================================
_i2c_write_verify:
    mov  r6, #200        ; try counter

_wv_loop:
    clr  0x81            ; SCL low
    lcall _i2c_delay_sled
    setb 0x81            ; SCL high
    lcall _i2c_delay_sled
    jnb  0x81, _wv_stretched  ; SCL held low by slave?
    djnz r6, _wv_loop

    ; timeout — no stretch detected
    mov  dpl, #0
    ret

_wv_stretched:
    mov  dpl, #1
    ret

; ============================================================
; void i2c_start(void)
; ============================================================
_i2c_start:
    setb 0x80            ; SDA high
    mov  0xB2, #0xFE
    lcall _i2c_delay_sled
    lcall _scl_high      ; SCL high (wait for stretch)
    lcall _i2c_delay_sled
    mov  0xB2, #0xFF     ; SDA low
    clr  0x80
    lcall _i2c_delay_sled
    clr  0x81            ; SCL low
    lcall _i2c_delay_sled
    ret

; ============================================================
; void i2c_stop(void)
; ============================================================
_i2c_stop:
    mov  0xB2, #0xFF     ; SDA low
    clr  0x80
    lcall _i2c_delay_sled
    lcall _scl_high      ; SCL high (wait for stretch)
    lcall _i2c_delay_sled
    setb 0x80            ; SDA high
    mov  0xB2, #0xFE
    lcall _i2c_delay_sled
    ret

; ============================================================
; void i2c_repeated_start(void)
; ============================================================
_i2c_repeated_start:
    setb 0x80            ; SDA high
    mov  0xB2, #0xFE
    lcall _i2c_delay_sled
    lcall _scl_high      ; SCL high (wait for stretch)
    lcall _i2c_delay_sled
    mov  0xB2, #0xFF     ; SDA low
    clr  0x80
    lcall _i2c_delay_sled
    clr  0x81            ; SCL low
    lcall _i2c_delay_sled
    ret

; ============================================================
; BOOL i2c_write_byte(BYTE b)
; Send 8 bits MSB-first, read ACK. Returns 1=ACK, 0=NAK.
; ============================================================
_i2c_write_byte:
    mov  a, dpl          ; byte to send
    mov  r6, #8          ; bit counter

_wb_loop:
    rlc  a               ; MSB -> carry
    mov  r5, a           ; save remaining bits
    jc   _wb_one
    ; --- bit = 0: SDA low ---
    mov  0xB2, #0xFF     ; OEA = all output
    clr  0x80            ; IOA.0 = 0
    sjmp _wb_clk
_wb_one:
    ; --- bit = 1: SDA high (release) ---
    setb 0x80            ; IOA.0 = 1 (latch high)
    mov  0xB2, #0xFE     ; OEA = SDA input
_wb_clk:
    lcall _scl_high      ; SCL high (wait for stretch)
    lcall _i2c_delay_sled
    clr  0x81            ; SCL low
    lcall _i2c_delay_sled
    mov  a, r5           ; restore byte
    djnz r6, _wb_loop

    ; --- ACK phase ---
    setb 0x80            ; release SDA (latch=1)
    mov  0xB2, #0xFE     ; SDA input
    lcall _scl_high      ; SCL high (wait for stretch)
    lcall _i2c_delay_sled
    mov  c, 0x80         ; read SDA into carry
    clr  0x81            ; SCL low
    lcall _i2c_delay_sled
    ; return !SDA: carry=0 (ACK) -> return 1, carry=1 (NAK) -> return 0
    clr  a
    cpl  c
    rlc  a
    mov  dpl, a
    ret

; ============================================================
; BYTE i2c_read_byte(BOOL ack)
; Read 8 bits MSB-first, send ACK/NAK. Returns byte read.
; ack in dpl: nonzero = send ACK, zero = send NAK.
; ============================================================
_i2c_read_byte:
    mov  r5, dpl         ; save ack flag
    ; release SDA
    setb 0x80
    mov  0xB2, #0xFE
    mov  r6, #8          ; bit counter
    clr  a               ; accumulator

_rb_loop:
    mov  r4, a           ; save accumulated byte
    lcall _scl_high      ; SCL high (wait for stretch)
    lcall _i2c_delay_sled
    ; read SDA and shift in
    mov  a, r4
    add  a, r4           ; a = r4 << 1
    mov  c, 0x80         ; carry = SDA
    addc a, #0           ; a = (r4 << 1) | SDA
    clr  0x81            ; SCL low
    lcall _i2c_delay_sled
    djnz r6, _rb_loop

    ; --- ACK/NAK phase ---
    mov  r4, a           ; save received byte
    mov  a, r5           ; ack flag
    jz   _rb_nak
    ; send ACK (SDA low)
    mov  0xB2, #0xFF
    clr  0x80
    sjmp _rb_ack_clk
_rb_nak:
    ; send NAK (SDA high)
    setb 0x80
    mov  0xB2, #0xFE
_rb_ack_clk:
    lcall _scl_high      ; SCL high (wait for stretch)
    lcall _i2c_delay_sled
    clr  0x81            ; SCL low
    lcall _i2c_delay_sled
    ; release SDA
    setb 0x80
    mov  0xB2, #0xFE
    mov  dpl, r4         ; return byte
    ret
