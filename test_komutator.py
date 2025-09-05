import serial
import time

SERIAL_PORT = 'COM13'  # Windows, замени на свой
BAUD_RATE = 115200

# CRC-8: Poly=0x31, Init=0xFF, RefIn=false, RefOut=false, XorOut=0x00
def crc8(data: bytes) -> int:
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_kp0(cycle: int) -> bytes:
    pkt = bytearray(4)
    pkt[0] = 0x00  # тип пакета КП1
    pkt[1] = (cycle >> 8) & 0xFF
    pkt[2] = cycle & 0xFF
    pkt[3] = crc8(pkt[:-1])
    return bytes(pkt)

def build_kp1(cycle: int, pair: int, sector: int) -> bytes:
    pkt = bytearray(6)
    pkt[0] = 0x01  # тип пакета КП1
    pkt[1] = (cycle >> 8) & 0xFF
    pkt[2] = cycle & 0xFF
    pkt[3] = pair
    pkt[4] = sector
    pkt[5] = crc8(pkt[:-1])
    return bytes(pkt)

def build_kp2(cycle: int, pair: int, time: int, sector: list[int]) -> bytes:
    pkt = bytearray(20)
    pkt[0] = 0x02  # тип пакета КП1
    pkt[1] = (cycle >> 8) & 0xFF
    pkt[2] = cycle & 0xFF
    pkt[3] = pair
    pkt[4] = (time >> 8) & 0xFF
    pkt[5] = time & 0xFF
    for i in range(len(sector)):
        pkt[i+6] = sector[i]
    pkt[19] = crc8(pkt[:-1])
    return bytes(pkt)

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
            time.sleep(2)  # время на инициализацию Arduino

            cycle = 0
            sector_int = 0
            pair = 3   # для примера управляем только парой 2
            period = 0xaabb
            sector_array = [0,1,2,3,4,5,6,6,5,4,3,2,1]

            while True:
                # Генерация КП1
                kp1 = build_kp1(cycle, pair, sector_int)
                ser.write(kp1)
                print(f"Sent KP1: {[f'0x{b:02X}' for b in kp1]}")

                response = ser.read(8)
                # if len(response) == 8:
                print(f"Received OP0: {[f'0x{b:02X}' for b in response]}")

                # Следующий цикл 1
                cycle = (cycle + 1) & 0xFFFF
                sector_int = (sector_int + 1) % 7  # сектора 0..6

                kp2 = build_kp2(cycle, pair, period, sector_array)
                ser.write(kp2)
                print(f"Sent KP2: {[f'0x{b:02X}' for b in kp2]}")

                # Ждем ответ (ОП0, 8 байт)
                response = ser.read(8)
                # if len(response) == 8:
                print(f"Received OP0: {[f'0x{b:02X}' for b in response]}")

                # Следующий цикл 2
                cycle = (cycle + 1) & 0xFFFF
                sector_array.pop(0)
                sector_array.append(sector_int)

                kp0 = build_kp0(cycle)
                ser.write(kp0)
                print(f"Sent KP0: {[f'0x{b:02X}' for b in kp0]}")

                # Ждем ответ (ОП0, 8 байт)
                response = ser.read(8)
                # if len(response) == 8:
                print(f"Received OP0: {[f'0x{b:02X}' for b in response]}")

                # Следующий цикл 2
                cycle = (cycle + 1) & 0xFFFF


    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except KeyboardInterrupt:
        print("\nProgram stopped")

if __name__ == "__main__":
    main()
