from serial.tools import list_ports

ports = list_ports.comports()

for port in ports:
    print(f"Device: {port.device}, VID: {port.vid}, PID: {port.pid}, Description: {port.description}")
