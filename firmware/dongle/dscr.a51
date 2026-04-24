; USB descriptors for Realtek USB-I2C adapter (open-source replacement)
; VID:PID = 2007:0808 (protocol-compatible with original realsil.hex)

.module DEV_DSCR

DSCR_DEVICE_TYPE=1
DSCR_CONFIG_TYPE=2
DSCR_STRING_TYPE=3
DSCR_INTERFACE_TYPE=4
DSCR_ENDPOINT_TYPE=5
DSCR_DEVQUAL_TYPE=6

DSCR_INTERFACE_LEN=9
DSCR_ENDPOINT_LEN=7

ENDPOINT_TYPE_BULK=2

    .globl _dev_dscr, _dev_qual_dscr, _highspd_dscr, _fullspd_dscr, _dev_strings, _dev_strings_end
    .area DSCR_AREA (CODE)

_dev_dscr:
    .db dev_dscr_end-_dev_dscr  ; len
    .db DSCR_DEVICE_TYPE        ; type
    .dw 0x0002                  ; usb 2.0
    .db 0xff                    ; class (vendor specific)
    .db 0xff                    ; subclass
    .db 0xff                    ; protocol
    .db 64                      ; max packet (ep0)
    .dw 0x0720                  ; vendor id  0x2007 (byte-swapped for little-endian)
    .dw 0x0808                  ; product id 0x0808
    .dw 0x0100                  ; version 1.00
    .db 1                       ; manufacturer string idx
    .db 2                       ; product string idx
    .db 0                       ; serial number string idx
    .db 1                       ; num configurations
dev_dscr_end:

_dev_qual_dscr:
    .db dev_qual_dscr_end-_dev_qual_dscr
    .db DSCR_DEVQUAL_TYPE
    .dw 0x0002                  ; usb 2.0
    .db 0xff
    .db 0xff
    .db 0xff
    .db 64                      ; max packet
    .db 1                       ; num configs
    .db 0                       ; reserved
dev_qual_dscr_end:

; --- High-speed config (USB 2.0 HS) ---
_highspd_dscr:
    .db highspd_dscr_end-_highspd_dscr
    .db DSCR_CONFIG_TYPE
    .db (highspd_dscr_realend-_highspd_dscr) % 256
    .db (highspd_dscr_realend-_highspd_dscr) / 256
    .db 1                       ; num interfaces
    .db 1                       ; config number
    .db 0                       ; config string
    .db 0x80                    ; bus powered
    .db 0x32                    ; max power 100mA
highspd_dscr_end:

    .db DSCR_INTERFACE_LEN
    .db DSCR_INTERFACE_TYPE
    .db 0                       ; interface 0
    .db 0                       ; alt setting 0
    .db 2                       ; num endpoints
    .db 0xff                    ; vendor class
    .db 0xff
    .db 0xff
    .db 3                       ; interface string idx

; EP4 OUT bulk (command input from host)
    .db DSCR_ENDPOINT_LEN
    .db DSCR_ENDPOINT_TYPE
    .db 0x04                    ; EP4 OUT
    .db ENDPOINT_TYPE_BULK
    .db 0x00                    ; max packet LSB
    .db 0x02                    ; max packet 512
    .db 0x00                    ; interval

; EP8 IN bulk (response output to host)
    .db DSCR_ENDPOINT_LEN
    .db DSCR_ENDPOINT_TYPE
    .db 0x88                    ; EP8 IN
    .db ENDPOINT_TYPE_BULK
    .db 0x00                    ; max packet LSB
    .db 0x02                    ; max packet 512
    .db 0x00                    ; interval

highspd_dscr_realend:

; --- Full-speed config (USB 1.1 fallback) ---
    .even
_fullspd_dscr:
    .db fullspd_dscr_end-_fullspd_dscr
    .db DSCR_CONFIG_TYPE
    .db (fullspd_dscr_realend-_fullspd_dscr) % 256
    .db (fullspd_dscr_realend-_fullspd_dscr) / 256
    .db 1
    .db 1
    .db 0
    .db 0x80
    .db 0x32
fullspd_dscr_end:

    .db DSCR_INTERFACE_LEN
    .db DSCR_INTERFACE_TYPE
    .db 0
    .db 0
    .db 2
    .db 0xff
    .db 0xff
    .db 0xff
    .db 3

; EP4 OUT bulk
    .db DSCR_ENDPOINT_LEN
    .db DSCR_ENDPOINT_TYPE
    .db 0x04
    .db ENDPOINT_TYPE_BULK
    .db 0x40                    ; max packet 64
    .db 0x00
    .db 0x00

; EP8 IN bulk
    .db DSCR_ENDPOINT_LEN
    .db DSCR_ENDPOINT_TYPE
    .db 0x88
    .db ENDPOINT_TYPE_BULK
    .db 0x40                    ; max packet 64
    .db 0x00
    .db 0x00

fullspd_dscr_realend:

; --- String descriptors ---
    .even
_dev_strings:

_string0:
    .db string0end-_string0
    .db DSCR_STRING_TYPE
    .db 0x09, 0x04              ; English (US)
string0end:

_string1:
    .db string1end-_string1
    .db DSCR_STRING_TYPE
    .ascii 'O'
    .db 0
    .ascii 'p'
    .db 0
    .ascii 'e'
    .db 0
    .ascii 'n'
    .db 0
    .ascii 'R'
    .db 0
    .ascii 'T'
    .db 0
    .ascii 'D'
    .db 0
string1end:

_string2:
    .db string2end-_string2
    .db DSCR_STRING_TYPE
    .ascii 'U'
    .db 0
    .ascii 'S'
    .db 0
    .ascii 'B'
    .db 0
    .ascii '-'
    .db 0
    .ascii 'I'
    .db 0
    .ascii '2'
    .db 0
    .ascii 'C'
    .db 0
string2end:

_string3:
    .db string3end-_string3
    .db DSCR_STRING_TYPE
    .ascii 'I'
    .db 0
    .ascii '2'
    .db 0
    .ascii 'C'
    .db 0
string3end:

_dev_strings_end:
    .dw 0x0000

; Bootloader device descriptor: 04b4:8613 (Cypress EZ-USB FX2 bootloader).
; Served via SUDPTRH:L which requires word-aligned (even) addresses.
    .even
    .globl _bl_dev_dscr
_bl_dev_dscr:
    .db 18          ; bLength
    .db 1           ; bDescriptorType (device)
    .dw 0x0002      ; bcdUSB 2.00
    .db 0xff        ; bDeviceClass (vendor)
    .db 0xff        ; bDeviceSubClass
    .db 0xff        ; bDeviceProtocol
    .db 64          ; bMaxPacketSize0
    .dw 0xB404      ; idVendor  0x04B4 (byte-swapped)
    .dw 0x1386      ; idProduct 0x8613 (byte-swapped)
    .dw 0x0001      ; bcdDevice 1.00
    .db 0           ; iManufacturer
    .db 0           ; iProduct
    .db 0           ; iSerialNumber
    .db 1           ; bNumConfigurations
