import serial
import numpy as np

def read_data_packet(ser):
    while True:
        if ser.read() == b'\x7E':  # Check for starting bit
            sensor_id = int.from_bytes(ser.read(), byteorder='little')  # Read sensor ID
            zone = int.from_bytes(ser.read(), byteorder='little')  # Read zone
            distance = int.from_bytes(ser.read(2), byteorder='little')  # Read distance (2 bytes)
            if ser.read() == b'\x7F':  # Check for ending bit
                return sensor_id, zone, distance

def print_matrices(matrix1, matrix2):
    print("Top Sensor\t\t\tBottom Sensor")
    for i in range(8):
        for j in range(8):
            print(f"{matrix1[i, j]:5}", end=" ")
        print("\t", end="")
        for j in range(8):
            print(f"{matrix2[i, j]:5}", end=" ")
        print()
    print()

def main():
    ser = serial.Serial('/dev/ttyACM1', 2000000)
    
    # Initialize matrices for top and bottom sensors
    matrix_top = np.zeros((8, 8), dtype=int)
    matrix_bottom = np.zeros((8, 8), dtype=int)
    
    try:
        while True:
            # Reset matrices
            matrix_top.fill(0)
            matrix_bottom.fill(0)
            
            for _ in range(64):  # Read 64 zones for both sensors
                sensor_id, zone, distance = read_data_packet(ser)
                row, col = divmod(zone, 8)
                if sensor_id == 1:
                    matrix_top[row, col] = distance
                elif sensor_id == 2:
                    matrix_bottom[row, col] = distance

            print_matrices(matrix_top, matrix_bottom)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
