import pygmdl

# Your center point (e.g., your test field)
# BIDC
# LAT = 40.42773956363986
# LON = -86.91873017427434
# ZOOM = 21    # 1 (World) to 21 (Street level)
# SIZE = 50  # Size of the area in meters

# GP track
# LAT = 40.43769648
# LON = -86.94443853
LAT = 40.43785675963577
LON= -86.94426683849075
SIZE = 145 
ZOOM = 20    # 1 (World) to 21 (Street level)

# This saves a high-res satellite PNG to your disk
pygmdl.save_image(LAT, LON, SIZE, "gp_map3.png", zoom=ZOOM, from_center=True)

print("Map downloaded for offline use.")