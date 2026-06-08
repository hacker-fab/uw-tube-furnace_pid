import serial, time
s = serial.Serial('/dev/cu.usbserial-A9XUD4EB', 9600, bytesize=7, parity='E', stopbits=1, timeout=1)
payload = '010310000001'
lrc = hex((0x100 - (sum(int(payload[i:i+2], 16) for i in range(0, len(payload), 2)) & 0xFF)) & 0xFF)[2:].upper().zfill(2)
s.write(f':{payload}{lrc}\r\n'.encode('ascii'))
time.sleep(0.1)
resp = s.read(100)
print(f'{int(resp[7:11], 16) / 10.0}°C')
s.close()
