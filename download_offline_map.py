import contextily as cx
                                                           
# (west, south, east, north) — includes Purdue campus and the Grand Prix Kartway    
cx.bounds2raster(-86.9450, 40.4100, -86.8800, 40.4695429,                              
                'maps/wlaf_z18.tif',                                               
                zoom=18,                                                           
                source=cx.providers.Esri.WorldImagery,                             
                ll=True)                                                           
print('saved maps/wlaf_z18.tif')
