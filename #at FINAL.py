#AT modified code
from pymavlink import mavutil
import math
import serial
import time
#gcs input
GCS_LAT=13.0308427
GCS_LON=77.5655385
GCS_ALT=945.91
print("Connecting to drone via pymavlink...")
master = mavutil.mavlink_connection('COM10',57600 )#MAVLINK CONNECTION OBJECT
master.wait_heartbeat()
print("Heartbeat received.")

print("Connecting to Arduino...")
arduino = serial.Serial('COM7', 9600, timeout=1)
time.sleep(2)
def servo_set(azi):
    if(270<=azi<=360):
        pan_set=360-azi
    elif(0<azi<90):
        pan_set=90+azi
    return(pan_set)
    
   
 
def calculate_angles(lat1, lon1, alt1, lat2, lon2, alt2):
    R = 6371000  # Earth radius in meters

    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    x = math.cos(lat2) * math.sin(dlon)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)

    azimuth = math.atan2(x, y)
    azimuth_deg = (math.degrees(azimuth) + 360) % 360 #returns the true azimuth angle wrt to north

    horizontal_dist = math.sqrt((R * dlat) ** 2 + (R * math.cos(lat1) * dlon) ** 2)
    height = alt2 - alt1
    elevation = math.atan2(height, horizontal_dist)
    elevation_deg = math.degrees(elevation)
    
    return azimuth_deg, elevation_deg
def send_to_arduino(pan, tilt):
    pan = int(max(0, min(180, pan)))#code to clamp angles 
    tilt = int(max(0, min(180, tilt)))#code to clamp angles
    command = f"{pan},{tilt}\n"
    arduino.write(command.encode())
    print(f"Sent: {command.strip()}")
try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg is None:
            time.sleep(0.05)  # Sleep 50ms and retry
            continue

        drone_lat = msg.lat / 1e7
        drone_lon = msg.lon / 1e7
        drone_alt = msg.alt / 1000.0

        azimuth, elevation = calculate_angles(GCS_LAT, GCS_LON, GCS_ALT, drone_lat, drone_lon, drone_alt)
        send_to_arduino(servo_set(azimuth),0)
       # q=servo_set(azimuth)
        #send_to_arduino(q,0)
        
        pan_servo = int(90 - math.sin(math.radians(azimuth)) * 90)
        tilt_servo = int(min(max(elevation, 0), 180))

        if 90 < azimuth < 270:
            tilt_servo = int(180 - tilt_servo)
            pan_servo = int(180 - pan_servo)

        send_to_arduino(pan_servo, tilt_servo)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    arduino.close()
    pass

