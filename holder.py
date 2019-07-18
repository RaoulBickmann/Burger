      # if(abs(diff) > 0.000001):
        
            # p = np.array(position[0], position[1])
            # x, y = 0, 0 
            
            # x,y = rotate((position[0] + 0.065, position[1]), (position[0], position[1]), angle)

            # b =  np.array([x, y, angle])
            # position = np.add(position, b)
            # position = np.array([x + position[0], y + position[1], total])
            # print(x, y)
            # print(position)
            
            
            
def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """    
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

